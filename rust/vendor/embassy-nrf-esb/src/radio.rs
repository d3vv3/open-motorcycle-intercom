//! RADIO register access layer for ESB.
//!
//! Thin abstraction over nRF RADIO peripheral registers, adapted from
//! esb-ng `src/peripherals.rs` for nrf-pac 0.3.
//!
//! Portions adapted from [esb-ng](https://github.com/jamesmunns/esb)
//! (MIT OR Apache-2.0). See `NOTICE.md` for attribution.
//!
//! This module handles raw register I/O only. Protocol logic (state machine,
//! duplicate detection, grant management) lives in `state_machine.rs`.

use core::cell::UnsafeCell;
use core::sync::atomic::{Ordering, compiler_fence};

use crate::addresses::EsbAddresses;
use crate::config::{Bitrate, EsbConfig};

#[cfg(feature = "fast-ru")]
use crate::pac::radio::vals::Ru;
use crate::pac::radio::vals::{Crcstatus, Endian, Len, Mode, Plen, Skipaddr, State as RadioState};
use crate::pac::radio::{Radio, regs};

/// Number of ESB pipes (matching hardware RXMATCH field width).
const NUM_PIPES: usize = 8;
/// Bounded spin count for RADIO disable.
///
/// Normal disable completion is a short hardware transition. If this bound is
/// hit, the radio state is already abnormal; power-cycle recovery is preferable
/// to wedging firmware forever.
const RADIO_DISABLE_SPIN_LIMIT: u32 = 100_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum StopOutcome {
    Normal,
    PowerCycled,
}

/// Pure register construction, shared with host tests (no MMIO).
fn packet_config(bitrate: Bitrate) -> regs::Pcnf0 {
    let mut value = regs::Pcnf0(0);
    value.set_lflen(8);
    value.set_s1len(3);
    // NCS v3.4.1 update_rf_payload_format_esb_dpl selects 16 bits at 2 Mbps.
    value.set_plen(match bitrate {
        Bitrate::Mbps1 => Plen::_8BIT,
        Bitrate::Mbps2 => Plen::_16BIT,
    });
    value
}

fn crc_polynomial(config: &EsbConfig) -> u32 {
    (config.crc.poly as u32) | (1u32 << (config.crc.length as u32 * 8))
}

fn receive_shortcuts(mut value: regs::Shorts, auto_ack: bool) -> regs::Shorts {
    value.set_disabled_txen(auto_ack);
    value.set_disabled_rxen(false);
    value
}

fn transmit_shortcuts(mut value: regs::Shorts, ack: bool) -> regs::Shorts {
    value.set_disabled_rxen(ack);
    value.set_disabled_txen(false);
    value
}

/// Result of checking a received PRX packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum RxResult {
    /// Stop reset RADIO; task service must recover without restarting DMA.
    RadioFault,
    /// New valid packet received on the given pipe.
    NewPacket,
    /// CRC check failed — radio automatically restarted.
    BadCrc,
}

/// RADIO register access layer for ESB.
///
/// Wraps `pac::radio::Radio` (Copy pointer). Tracks per-pipe CRC and PID
/// for duplicate detection (PRX).
#[allow(dead_code)]
pub(crate) struct EsbRadio {
    radio: Radio,
    receive_auto_ack: bool,
    stop_fault: bool,
    /// Last received CRC per pipe, for duplicate detection.
    last_crc: [u16; NUM_PIPES],
    /// Last received PID per pipe, for duplicate detection.
    last_pid: [u8; NUM_PIPES],
    /// Whether the duplicate-detection entry for each pipe is initialized.
    last_valid: [bool; NUM_PIPES],
}

#[allow(dead_code)]
impl EsbRadio {
    /// Create a new radio layer from the PAC RADIO instance.
    pub(crate) const fn new(radio: Radio) -> Self {
        Self {
            radio,
            receive_auto_ack: false,
            stop_fault: false,
            last_crc: [0; NUM_PIPES],
            last_pid: [0; NUM_PIPES],
            last_valid: [false; NUM_PIPES],
        }
    }

    /// Full RADIO register initialization for ESB mode.
    ///
    /// Ref: esb-ng `peripherals.rs` lines 83–143.
    /// All register writes follow esb-ng exactly unless noted.
    pub(crate) fn init(&mut self, config: &EsbConfig, addresses: &EsbAddresses) {
        let r = self.radio;

        self.receive_auto_ack = config.receive_auto_ack;

        // Clear all interrupts (esb-ng line 85).
        r.intenclr().write_value(regs::Int(0xFFFF_FFFF));

        // Data rate (esb-ng line 86).
        let mode = match config.bitrate {
            Bitrate::Mbps1 => Mode::NRF_1MBIT,
            Bitrate::Mbps2 => Mode::NRF_2MBIT,
        };
        r.mode().write(|w| w.set_mode(mode));

        // Convert addresses for nRF24L01+ compatibility (esb-ng lines 89–92).
        // Bit-reversal for base, bytewise bit-swap for prefixes.
        let base0 = addresses.base0_reg();
        let base1 = addresses.base1_reg();
        let prefix0 = addresses.prefix0_reg();
        let prefix1 = addresses.prefix1_reg();

        // Configure shortcuts (esb-ng lines 94–99).
        r.shorts().write(|w| {
            w.set_ready_start(true); // READY → START
            w.set_end_disable(true); // END → DISABLE
            w.set_address_rssistart(true); // ADDRESS → RSSI START
            w.set_disabled_rssistop(true); // DISABLED → RSSI STOP
        });

        // Fast ramp-up (PS §6.17.10 MODECNF0.RU).
        #[cfg(feature = "fast-ru")]
        r.modecnf0().modify(|w| w.set_ru(Ru::FAST));

        // TX output power (esb-ng line 105).
        r.txpower()
            .write(|w| w.set_txpower(config.tx_power.to_pac()));

        // PCNF0: 8-bit LENGTH + S1 field (PID + ACK = !noack).
        // S1LEN=3: bits 2:1 = PID, bit 0 = ACK.
        // S1INCL not set: relies on reset default (Automatic = include when S1LEN>0).
        // DO NOT change S1INCL without updating DMA buffer layout (R16).
        r.pcnf0().write_value(packet_config(config.bitrate));

        // PCNF1: max payload, 4-byte base + 1-byte prefix, big-endian
        // (esb-ng lines 112–120). MAXLEN mirrors software validation so the
        // RADIO cannot accept or DMA a packet larger than the configured
        // maximum payload length.
        r.pcnf1().write(|w| {
            w.set_maxlen(config.payload_length);
            w.set_balen(4); // 4-byte base address
            w.set_statlen(0);
            w.set_endian(Endian::BIG);
        });

        // CRC configuration (esb-ng lines 122–130).
        // SKIPADDR=INCLUDE: address included in CRC (nRF24L01+ compatible).
        let crc_len = match config.crc.length {
            0 => Len::DISABLED,
            1 => Len::ONE,
            _ => Len::TWO,
        };
        // For N-byte CRC, the polynomial MSB is bit (N*8). Config stores lower bits.
        let crc_poly = crc_polynomial(config);
        r.crcinit()
            .write(|w| w.set_crcinit(config.crc.init as u32 & 0x00FF_FFFF));
        r.crcpoly().write(|w| w.set_crcpoly(crc_poly & 0x00FF_FFFF));
        r.crccnf().write(|w| {
            w.set_len(crc_len);
            w.set_skipaddr(Skipaddr::INCLUDE);
        });

        // Write addresses (esb-ng lines 132–136).
        r.base0().write_value(base0);
        r.base1().write_value(base1);
        r.prefix0().write_value(regs::Prefix0(prefix0));
        r.prefix1().write_value(regs::Prefix1(prefix1));

        // Set RF channel (esb-ng lines 140–142).
        // Channel maps to 2400 + channel MHz.
        r.frequency().write(|w| w.set_frequency(config.channel.0));
    }

    // ---- Event management ----

    /// Clear EVENTS_DISABLED to prevent retrigger (esb-ng line 148).
    #[inline]
    pub(crate) fn clear_disabled_event(&mut self) {
        self.radio.events_disabled().write_value(0);
    }

    /// Check if EVENTS_DISABLED is set (esb-ng line 153).
    #[inline]
    pub(crate) fn check_disabled_event(&self) -> bool {
        self.radio.events_disabled().read() == 1
    }

    /// Read EVENTS_END without consuming it; TX handoff clears stale END.
    #[inline]
    pub(crate) fn check_end_event(&self) -> bool {
        self.radio.events_end().read() == 1
    }

    /// Clear EVENTS_END (esb-ng line 159).
    #[inline]
    pub(crate) fn clear_end_event(&mut self) {
        self.radio.events_end().write_value(0);
    }

    /// Check EVENTS_READY (esb-ng line 165).
    #[inline]
    pub(crate) fn check_ready_event(&self) -> bool {
        self.radio.events_ready().read() == 1
    }

    /// Clear EVENTS_READY (esb-ng line 170).
    #[inline]
    pub(crate) fn clear_ready_event(&mut self) {
        self.radio.events_ready().write_value(0);
    }

    /// Disable the DISABLED interrupt (esb-ng line 177).
    #[inline]
    pub(crate) fn disable_disabled_interrupt(&mut self) {
        self.radio.intenclr().write(|w| w.set_disabled(true));
    }

    // ---- Stop / disable ----

    /// Stop the radio: disable shortcuts, trigger TASKS_DISABLE, bounded wait.
    /// Ref: esb-ng lines 182–203.
    pub(crate) fn stop(&mut self) -> StopOutcome {
        let r = self.radio;

        // Clear mode-switching shortcuts before disabling.
        r.shorts().modify(|w| {
            w.set_disabled_rxen(false);
            w.set_disabled_txen(false);
        });
        self.disable_disabled_interrupt();
        // NoAck END already disabled the peripheral; do not wait for a second
        // DISABLED event or power-cycle away the received packet's metadata.
        if r.state().read().state() == RadioState::DISABLED {
            self.clear_disabled_event();
            compiler_fence(Ordering::Acquire);
            return StopOutcome::Normal;
        }
        self.clear_disabled_event();
        r.tasks_disable().write_value(1);

        // Wait for EVENTS_DISABLED (esb-ng line 192), but never wedge the
        // firmware forever if the peripheral is in an unexpected state.
        let mut spins = 0;
        while r.events_disabled().read() == 0 && spins < RADIO_DISABLE_SPIN_LIMIT {
            spins += 1;
        }
        let outcome = if spins == RADIO_DISABLE_SPIN_LIMIT || !self.is_quiescent() {
            r.power().write(|w| w.set_power(false));
            r.power().write(|w| w.set_power(true));
            self.stop_fault = true;
            StopOutcome::PowerCycled
        } else {
            StopOutcome::Normal
        };
        self.clear_disabled_event();

        // Ensure DMA writes are visible to CPU (esb-ng line 196).
        compiler_fence(Ordering::Acquire);
        outcome
    }

    pub(crate) fn is_quiescent(&self) -> bool {
        self.radio.state().read().state() == RadioState::DISABLED
    }

    pub(crate) fn take_stop_fault(&mut self) -> bool {
        core::mem::take(&mut self.stop_fault)
    }

    pub(crate) fn has_stop_fault(&self) -> bool {
        self.stop_fault
    }

    /// Called only after quiescence, before publishing another role/epoch.
    pub(crate) fn clear_all_events(&mut self) {
        let r = self.radio;
        r.events_ready().write_value(0);
        r.events_address().write_value(0);
        r.events_payload().write_value(0);
        r.events_end().write_value(0);
        r.events_disabled().write_value(0);
        r.events_devmatch().write_value(0);
        r.events_devmiss().write_value(0);
        r.events_rssiend().write_value(0);
        r.events_bcmatch().write_value(0);
        r.events_crcok().write_value(0);
        r.events_crcerror().write_value(0);
        #[cfg(any(feature = "nrf52840", feature = "nrf52833"))]
        {
            r.events_framestart().write_value(0);
            r.events_edend().write_value(0);
            r.events_edstopped().write_value(0);
            r.events_ccaidle().write_value(0);
            r.events_ccabusy().write_value(0);
            r.events_ccastopped().write_value(0);
            r.events_rateboost().write_value(0);
            r.events_txready().write_value(0);
            r.events_rxready().write_value(0);
            r.events_mhrmatch().write_value(0);
            r.events_sync().write_value(0);
            r.events_phyend().write_value(0);
        }
    }

    // ---- PTX methods ----

    /// Start a TX transmission.
    ///
    /// Sets up RADIO for transmit on the given pipe. If `ack` is true,
    /// the radio will automatically transition to RX mode after TX END
    /// via the `disabled_rxen` shortcut (esb-ng lines 208–237).
    ///
    /// `dma_ptr` points to the DMA region (EsbHeader offset 2) of the TX buffer.
    pub(crate) fn transmit(&mut self, pipe: u8, dma_ptr: *mut u8, ack: bool) {
        self.prepare_transmit(pipe, dma_ptr, ack);
        self.launch_transmit_cpu();
    }

    /// Prepare stable DMA/register state without issuing TXEN.
    pub(crate) fn prepare_transmit(&mut self, pipe: u8, dma_ptr: *mut u8, ack: bool) {
        let r = self.radio;

        // Set both turnaround bits on every TX, including the NoAck path.
        r.shorts().modify(|w| *w = transmit_shortcuts(*w, ack));
        // After TX END → DISABLED → auto-enable RX (esb-ng line 211).
        // NoAck clears disabled_rxen, including stale values from previous TX.
        // Radio goes DISABLED after TX END, release buffer immediately.

        // Enable DISABLED interrupt (esb-ng line 213).
        r.intenset().write(|w| w.set_disabled(true));

        // Set TX and RX addresses for this pipe (esb-ng lines 215–221).
        r.txaddress().write(|w| w.set_txaddress(pipe));
        r.rxaddresses().write_value(regs::Rxaddresses(1 << pipe));

        // Set DMA pointer (esb-ng line 223).
        r.packetptr().write_value(dma_ptr as u32);

        // Clear events BEFORE triggering task (esb-ng lines 226–230).
        // If events are already set, shortcuts fire immediately.
        r.events_address().write_value(0);
        self.clear_disabled_event();
        self.clear_ready_event();
        self.clear_end_event();
        r.events_payload().write_value(0);

        // Ensure DMA buffer writes are visible to RADIO before TXEN
        // (esb-ng line 233).
        compiler_fence(Ordering::Release);
        #[cfg(target_arch = "arm")]
        cortex_m::asm::dmb();
    }

    pub(crate) fn launch_transmit_cpu(&mut self) {
        // Start TX ramp-up (esb-ng line 235).
        self.radio.tasks_txen().write_value(1);
    }

    /// Finish NoAck TX only after current TX END and DISABLED, then quiescence.
    /// The DISABLED interrupt should be disabled after this.
    /// Ref: esb-ng lines 240–252.
    pub(crate) fn finish_tx_no_ack(&mut self) {
        // Ensure all DMA writes are visible (esb-ng line 243).
        compiler_fence(Ordering::SeqCst);

        // Disable interrupt — will be re-enabled in next transmit() call.
        self.disable_disabled_interrupt();
    }

    /// Set up RX buffer for ACK reception after TX.
    ///
    /// Must be called immediately after TX END event, before RADIO ramps up
    /// to RX (the `disabled_rxen` shortcut triggers RXEN automatically).
    /// Ref: esb-ng lines 256–274.
    pub(crate) fn prepare_for_ack(&mut self, dma_ptr: *mut u8) {
        let r = self.radio;

        self.clear_ready_event();

        // Ensure any prior DMA buffer writes are visible before PACKETPTR
        // (esb-ng line 261).
        compiler_fence(Ordering::Release);

        r.packetptr().write_value(dma_ptr as u32);

        // Verify radio hasn't already ramped up (timing assertion, esb-ng line 267).
        debug_assert!(!self.check_ready_event(), "Missed RX window (PTX)");

        // Clear the shortcut since we're now in RX mode (esb-ng line 271).
        r.shorts().modify(|w| w.set_disabled_rxen(false));
    }

    /// Check ACK CRC status after RX.
    /// Returns `true` if CRC passed (ACK received successfully).
    /// Ref: esb-ng lines 279–282.
    #[inline]
    pub(crate) fn check_ack(&self) -> bool {
        let ok = self.radio.crcstatus().read().crcstatus() == Crcstatus::CRCOK;
        // Ensure DMA writes from RADIO are visible to CPU (esb-ng line 282).
        compiler_fence(Ordering::Acquire);
        ok
    }

    // ---- PRX methods ----

    /// Start listening for packets in PRX mode.
    ///
    /// Sets up RADIO for RX on the given pipe bitmask. After receiving a
    /// valid packet, `disabled_txen` auto-transitions to ACK TX only when
    /// receive_auto_ack is enabled.
    /// Ref: esb-ng lines 309–333.
    pub(crate) fn start_receiving(&mut self, enabled_pipes: u8, dma_ptr: *mut u8) {
        let r = self.radio;

        // After RX END → DISABLED → auto-enable TX for ACK (esb-ng line 311).
        r.shorts()
            .modify(|w| *w = receive_shortcuts(*w, self.receive_auto_ack));

        r.intenset().write(|w| w.set_disabled(true));
        r.rxaddresses()
            .write_value(regs::Rxaddresses(enabled_pipes as u32));

        r.packetptr().write_value(dma_ptr as u32);

        // Clear events BEFORE triggering task (esb-ng lines 321–325).
        r.events_address().write_value(0);
        self.clear_disabled_event();
        self.clear_ready_event();
        self.clear_end_event();
        r.events_payload().write_value(0);

        // Ensure DMA buffer writes visible to RADIO (esb-ng line 328).
        compiler_fence(Ordering::Release);

        // Start RX ramp-up (esb-ng line 330).
        r.tasks_rxen().write_value(1);
    }

    /// Start listening without the automatic RX->TX ACK shortcut.
    ///
    /// The MPSL PRX path uses this for multi-pipe ACKs: it must read RXMATCH
    /// and program TXADDRESS before starting ACK TX. With disabled_txen, the
    /// radio can begin ACK TX before software switches away from pipe 0.
    #[cfg(feature = "mpsl")]
    pub(crate) fn start_receiving_manual_ack(&mut self, enabled_pipes: u8, dma_ptr: *mut u8) {
        let r = self.radio;

        r.shorts().modify(|w| {
            w.set_disabled_txen(false);
            w.set_disabled_rxen(false);
        });
        r.intenset().write(|w| w.set_disabled(true));
        r.rxaddresses()
            .write_value(regs::Rxaddresses(enabled_pipes as u32));
        r.packetptr().write_value(dma_ptr as u32);

        r.events_address().write_value(0);
        self.clear_disabled_event();
        self.clear_ready_event();
        self.clear_end_event();
        r.events_payload().write_value(0);

        compiler_fence(Ordering::Release);
        r.tasks_rxen().write_value(1);
    }

    /// Start ACK TX after RXMATCH has selected the ACK pipe.
    #[cfg(feature = "mpsl")]
    pub(crate) fn transmit_ack_manual(&mut self, pipe: u8, ack_dma_ptr: *mut u8) {
        let r = self.radio;

        r.txaddress().write(|w| w.set_txaddress(pipe));
        r.shorts().modify(|w| {
            w.set_disabled_txen(false);
            w.set_disabled_rxen(false);
        });
        r.intenset().write(|w| w.set_disabled(true));
        r.packetptr().write_value(ack_dma_ptr as u32);

        r.events_address().write_value(0);
        self.clear_disabled_event();
        self.clear_ready_event();
        self.clear_end_event();
        r.events_payload().write_value(0);

        compiler_fence(Ordering::Release);
        r.tasks_txen().write_value(1);
    }

    /// Check received PRX packet CRC and read metadata.
    ///
    /// Returns `RxResult::BadCrc` if CRC failed and RX restarted. A stop reset
    /// instead returns RadioFault, retaining ownership until task recovery.
    /// On CRC OK, returns `RxResult::NewPacket` — the caller should then read
    /// PID from the DMA buffer and call `check_duplicate()` for full detection.
    ///
    /// On CRC failure, the radio is automatically restarted.
    /// Ref: esb-ng lines 337–357.
    pub(crate) fn check_packet(&mut self) -> RxResult {
        let r = self.radio;
        if self.stop_fault {
            return RxResult::RadioFault;
        }

        // Check CRC first (esb-ng line 344).
        if r.crcstatus().read().crcstatus() == Crcstatus::CRCERROR {
            // Bad CRC → restart RX without ACK (esb-ng lines 345–354).
            if self.stop() == StopOutcome::PowerCycled {
                return RxResult::RadioFault;
            }
            r.shorts()
                .modify(|w| *w = receive_shortcuts(*w, self.receive_auto_ack));
            r.intenset().write(|w| w.set_disabled(true));
            compiler_fence(Ordering::Release);
            r.tasks_rxen().write_value(1);
            return RxResult::BadCrc;
        }

        // CRC OK — ensure DMA writes visible (esb-ng line 357).
        // This fence also orders the caller's subsequent DMA buffer reads
        // (PID, payload) — they see the RADIO's DMA writes.
        compiler_fence(Ordering::Acquire);
        self.clear_ready_event();

        RxResult::NewPacket
    }

    /// Check if a received packet is a duplicate of the last packet on this pipe.
    ///
    /// Uses exact CRC + PID comparison (esb-ng line 364).
    /// Caller reads PID from DMA buffer after `check_packet()` returns NewPacket.
    pub(crate) fn check_duplicate(&self, pipe: usize, pid: u8, crc: u16) -> bool {
        pipe < NUM_PIPES
            && self.last_valid[pipe]
            && self.last_crc[pipe] == crc
            && self.last_pid[pipe] == pid
    }

    /// Update duplicate detection tracking for a pipe after accepting a new packet.
    /// Must be called after `check_duplicate()` returns false.
    pub(crate) fn update_detection(&mut self, pipe: usize, pid: u8, crc: u16) {
        if pipe < NUM_PIPES {
            self.last_crc[pipe] = crc;
            self.last_pid[pipe] = pid;
            self.last_valid[pipe] = true;
        }
    }

    /// Read RSSI sample from the radio (esb-ng line 293).
    #[inline]
    pub(crate) fn rssi_sample(&self) -> u8 {
        self.radio.rssisample().read().rssisample()
    }

    /// Read which pipe matched on the last RX (esb-ng line 360).
    #[inline]
    pub(crate) fn rx_match(&self) -> u8 {
        self.radio.rxmatch().read().rxmatch()
    }

    /// Read CRC from the last received packet (esb-ng line 361).
    #[inline]
    pub(crate) fn rx_crc(&self) -> u16 {
        self.radio.rxcrc().read().rxcrc() as u16
    }

    /// Set up ACK transmission in PRX mode.
    ///
    /// Configures TX address, sets DMA pointer for ACK payload.
    /// Must be called quickly after check_packet() — the radio is ramping
    /// to TX via the `disabled_txen` shortcut.
    /// Ref: esb-ng lines 366–404.
    pub(crate) fn setup_ack_tx(&mut self, pipe: u8, ack_dma_ptr: *mut u8) {
        let r = self.radio;

        // Set TX address to the pipe that received the packet
        // (esb-ng lines 373–375).
        r.txaddress().write(|w| w.set_txaddress(pipe));

        // Ensure ACK payload visible to RADIO (esb-ng line 401).
        compiler_fence(Ordering::Release);

        // Set DMA pointer for ACK packet (esb-ng line 404).
        r.packetptr().write_value(ack_dma_ptr as u32);

        // Verify ramp-up hasn't completed yet (esb-ng line 407).
        debug_assert!(!self.check_ready_event(), "Missed TX window (PRX)");

        // Swap shortcuts: after ACK TX → DISABLED → auto-enable RX
        // (esb-ng lines 412–415).
        r.shorts().modify(|w| {
            w.set_disabled_txen(false);
            w.set_disabled_rxen(true);
        });
    }

    /// Pointer to the shared 2-byte fallback empty-ACK buffer `[0, 0]`.
    ///
    /// Minimum ACK is 2 bytes: DMA needs valid length + pid_no_ack fields.
    fn fallback_ack_ptr(&self) -> *mut u8 {
        // SAFETY: UnsafeCell provides interior mutability for DMA access.
        // The RADIO only reads from this buffer (never writes) — it transmits
        // the 2-byte content as an empty ACK packet.
        //
        // SAFETY (Sync wrapper): Access is single-threaded — only called from
        // the RADIO ISR or the MPSL timeslot callback. No concurrent access.
        //
        // link_section(".data") guarantees RAM placement — EasyDMA cannot read
        // from Flash (errata [122], PS §6.17.6).
        #[repr(C, align(4))]
        struct FallbackAck(UnsafeCell<[u8; 2]>);
        unsafe impl Sync for FallbackAck {}
        #[unsafe(link_section = ".data")]
        static FALLBACK_ACK: FallbackAck = FallbackAck(UnsafeCell::new([0, 0]));
        // SAFETY: ISR-only access, RADIO reads while in TX mode.
        unsafe { (*FALLBACK_ACK.0.get()).as_mut_ptr() }
    }

    /// Set up ACK TX with fallback empty ACK `[0, 0]` (auto shortcut path).
    /// Used when no ACK payload is queued (esb-ng lines 377).
    pub(crate) fn setup_ack_tx_fallback(&mut self, pipe: u8) {
        let ptr = self.fallback_ack_ptr();
        self.setup_ack_tx(pipe, ptr);
    }

    /// Start ACK TX with the fallback empty ACK, manual (MPSL) turnaround.
    #[cfg(feature = "mpsl")]
    pub(crate) fn transmit_ack_manual_fallback(&mut self, pipe: u8) {
        let ptr = self.fallback_ack_ptr();
        self.transmit_ack_manual(pipe, ptr);
    }

    /// Stop PRX TX (NoAck path) — stops radio before TX begins
    /// (esb-ng lines 417–419).
    pub(crate) fn stop_prx_no_ack(&mut self) {
        self.stop();
    }

    /// Complete ACK transmission: set up next RX buffer and swap shortcuts.
    ///
    /// Called after ACK TX END event. Prepares radio for next RX.
    /// Ref: esb-ng lines 446–476.
    pub(crate) fn complete_rx_ack(&mut self, rx_dma_ptr: *mut u8) {
        let r = self.radio;

        compiler_fence(Ordering::SeqCst);

        r.packetptr().write_value(rx_dma_ptr as u32);

        // Swap shortcuts: disabled_rxen off (already hit), disabled_txen on
        // for next RX → ACK cycle (esb-ng lines 471–474).
        r.shorts().modify(|w| {
            w.set_disabled_rxen(false);
            w.set_disabled_txen(self.receive_auto_ack);
        });
    }

    /// Restart RX after handling a NoAck packet.
    /// Ref: esb-ng lines 480–497.
    pub(crate) fn complete_rx_no_ack(&mut self, rx_dma_ptr: *mut u8) {
        let r = self.radio;

        r.events_address().write_value(0);
        self.clear_disabled_event();
        self.clear_ready_event();
        self.clear_end_event();
        r.events_payload().write_value(0);
        r.packetptr().write_value(rx_dma_ptr as u32);

        r.shorts()
            .modify(|w| *w = receive_shortcuts(*w, self.receive_auto_ack));
        r.intenset().write(|w| w.set_disabled(true));
        compiler_fence(Ordering::Release);
        r.tasks_rxen().write_value(1);
    }

    // ---- Suspend / Resume helpers ----

    /// Reset duplicate detection state (for suspend/resume across MPSL timeslots).
    pub(crate) fn reset_detection_state(&mut self) {
        self.last_crc = [0; NUM_PIPES];
        self.last_pid = [0; NUM_PIPES];
        self.last_valid = [false; NUM_PIPES];
    }

    /// Save per-pipe PID state for suspend.
    pub(crate) fn save_pid_state(&self) -> [u8; NUM_PIPES] {
        self.last_pid
    }

    /// Restore per-pipe PID state after resume.
    pub(crate) fn restore_pid_state(&mut self, pid: [u8; NUM_PIPES]) {
        self.last_pid = pid;
    }

    /// Save per-pipe CRC state for suspend.
    pub(crate) fn save_crc_state(&self) -> [u16; NUM_PIPES] {
        self.last_crc
    }

    /// Restore per-pipe CRC state after resume.
    pub(crate) fn restore_crc_state(&mut self, crc: [u16; NUM_PIPES]) {
        self.last_crc = crc;
    }

    /// Save duplicate-detection valid bits for suspend.
    pub(crate) fn save_detection_valid_state(&self) -> [bool; NUM_PIPES] {
        self.last_valid
    }

    /// Restore duplicate-detection valid bits after resume.
    pub(crate) fn restore_detection_valid_state(&mut self, valid: [bool; NUM_PIPES]) {
        self.last_valid = valid;
    }

    /// Power-cycle the RADIO peripheral (for MPSL timeslot transitions).
    /// PS §6.17: POWER register resets all RADIO registers to initial values.
    pub(crate) fn power_cycle(&mut self) {
        let r = self.radio;
        r.power().write(|w| w.set_power(false));
        r.power().write(|w| w.set_power(true));
    }

    /// Get the underlying PAC Radio reference (for direct register access
    /// in exceptional cases, e.g., MPSL integration).
    #[inline]
    pub(crate) fn regs(&self) -> Radio {
        self.radio
    }
}

#[cfg(test)]
mod tests {
    use super::{EsbRadio, crc_polynomial, packet_config, receive_shortcuts, transmit_shortcuts};
    use crate::config::{Bitrate, EsbConfig};
    use crate::pac::radio::{regs, vals::Plen};

    #[test]
    fn packet_register_values_match_scoped_profile() {
        for (bitrate, preamble, raw) in [
            (Bitrate::Mbps1, Plen::_8BIT, 0x0003_0008),
            (Bitrate::Mbps2, Plen::_16BIT, 0x0103_0008),
        ] {
            let value = packet_config(bitrate);
            assert_eq!(value.plen(), preamble);
            assert_eq!(value.lflen(), 8);
            assert_eq!(value.s1len(), 3);
            assert!(!value.s0len());
            assert_eq!(value.0, raw);
        }
        let config = EsbConfig::default();
        assert_eq!(crc_polynomial(&config), 0x11021);
        assert_eq!(config.crc.init, 0xffff);
        assert_eq!(config.crc.length, 2);
    }

    #[test]
    fn turnaround_shortcuts_replace_stale_bits() {
        let mut stale = regs::Shorts(0);
        stale.set_ready_start(true);
        stale.set_end_disable(true);
        stale.set_disabled_txen(true);
        stale.set_disabled_rxen(true);
        for enabled in [false, true] {
            let rx = receive_shortcuts(stale, enabled);
            assert_eq!(rx.disabled_txen(), enabled);
            assert!(!rx.disabled_rxen());
            assert!(rx.ready_start() && rx.end_disable());
            let tx = transmit_shortcuts(stale, enabled);
            assert_eq!(tx.disabled_rxen(), enabled);
            assert!(!tx.disabled_txen());
            assert!(tx.ready_start() && tx.end_disable());
        }
    }

    #[test]
    fn duplicate_detection_requires_valid_entry() {
        let mut radio = EsbRadio::new(crate::pac::RADIO);

        assert!(!radio.check_duplicate(0, 0, 0));

        radio.update_detection(0, 0, 0);
        assert!(radio.check_duplicate(0, 0, 0));
        assert!(!radio.check_duplicate(0, 1, 0));
        assert!(!radio.check_duplicate(0, 0, 1));
        assert!(!radio.check_duplicate(8, 0, 0));
    }

    #[test]
    fn duplicate_detection_valid_bits_round_trip_through_save_restore() {
        let mut radio = EsbRadio::new(crate::pac::RADIO);
        radio.update_detection(2, 3, 0x1234);

        let pid = radio.save_pid_state();
        let crc = radio.save_crc_state();
        let valid = radio.save_detection_valid_state();

        let mut restored = EsbRadio::new(crate::pac::RADIO);
        restored.restore_pid_state(pid);
        restored.restore_crc_state(crc);
        restored.restore_detection_valid_state(valid);

        assert!(restored.check_duplicate(2, 3, 0x1234));
        assert!(!restored.check_duplicate(1, 3, 0x1234));
    }

    #[test]
    fn reset_detection_clears_valid_bits() {
        let mut radio = EsbRadio::new(crate::pac::RADIO);
        radio.update_detection(4, 2, 0xabcd);
        assert!(radio.check_duplicate(4, 2, 0xabcd));

        radio.reset_detection_state();
        assert!(!radio.check_duplicate(4, 2, 0xabcd));
    }
}
