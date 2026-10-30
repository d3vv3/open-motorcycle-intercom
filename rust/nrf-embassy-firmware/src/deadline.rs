//! Register-independent autonomous cutoff protocol. No physical timing claim.

pub const START_LEAD_US: u32 = 20;
pub const START_MARGIN_US: u32 = 2;
pub const MAX_REMAINING_US: u32 = i32::MAX as u32;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Unavailable,
    Expired,
    HardwareFault,
}

/// Implementations must capture TIMER4 BEFORE reading the shared TIMER0 clock.
/// Programming must finish well before the parked half-wrap compare; neither
/// this protocol nor its 20 us lead bounds CPU stalls or MMIO latency.
pub trait Registers {
    fn pair(&mut self) -> Option<(u32, u64)>;
    fn group_off(&mut self);
    fn group_on(&mut self);
    fn group_enabled(&mut self) -> bool;
    fn cutoff_route(&mut self, enabled: bool);
    fn deadline_interrupt(&mut self, enabled: bool);
    fn clear_pending(&mut self);
    fn event(&mut self, index: usize) -> bool;
    fn clear_event(&mut self, index: usize);
    fn write_cc(&mut self, index: usize, raw: u32);
    fn read_cc(&mut self, index: usize) -> u32;
    fn radio_disable(&mut self);
}

#[derive(Clone, Copy)]
struct Arm<E> {
    epoch: E,
    deadline: u64,
    raw: u32,
    launched: bool,
}

pub struct State<E> {
    active: Option<Arm<E>>,
    expired: Option<E>,
}

impl<E: Copy + Eq> Default for State<E> {
    fn default() -> Self {
        Self::new()
    }
}

impl<E: Copy + Eq> State<E> {
    pub const fn new() -> Self {
        Self {
            active: None,
            expired: None,
        }
    }

    pub fn expired(&mut self, io: &mut impl Registers, epoch: E) -> bool {
        self.expired == Some(epoch)
            || self
                .active
                .is_some_and(|arm| arm.epoch == epoch && (io.event(0) || !io.group_enabled()))
    }

    /// Copy only the matching active cutoff; no register or clock access.
    #[cfg(feature = "sync-trace")]
    pub fn raw_cutoff(&self, epoch: E) -> Option<u32> {
        self.active
            .filter(|arm| arm.epoch == epoch)
            .map(|arm| arm.raw)
    }

    pub fn disarm(&mut self, io: &mut impl Registers) {
        // Gate first, retaining a cutoff that was latched before event clearing.
        io.group_off();
        io.cutoff_route(false);
        io.deadline_interrupt(false);
        if let Some(arm) = self.active.take()
            && io.event(0)
        {
            self.expired = Some(arm.epoch);
        }
        io.clear_pending();
        for index in 0..6 {
            io.clear_event(index);
            let _ = io.event(index);
        }
    }

    fn fail(&mut self, io: &mut impl Registers, error: Error) -> Error {
        if error == Error::Expired {
            if let Some(arm) = self.active {
                self.expired = Some(arm.epoch);
            }
            io.group_off();
            io.radio_disable();
        }
        self.disarm(io);
        error
    }

    fn program(io: &mut impl Registers, index: usize, raw: u32) -> Result<(), Error> {
        io.clear_event(index);
        let _ = io.event(index);
        io.write_cc(index, raw);
        if io.read_cc(index) != raw {
            return Err(Error::HardwareFault);
        }
        io.clear_event(index);
        let _ = io.event(index);
        Ok(())
    }

    fn due(io: &mut impl Registers, arm: Arm<E>) -> Result<(u32, u64), Error> {
        let (raw, now) = io.pair().ok_or(Error::Unavailable)?;
        if io.event(0)
            || !io.group_enabled()
            || now >= arm.deadline
            || arm.raw.wrapping_sub(raw) == 0
            || arm.raw.wrapping_sub(raw) > MAX_REMAINING_US
        {
            return Err(Error::Expired);
        }
        Ok((raw, now))
    }

    pub fn arm(&mut self, io: &mut impl Registers, epoch: E, deadline: u64) -> Result<(), Error> {
        self.disarm(io);
        self.expired = None;
        let result = (|| {
            let (raw, now) = io.pair().ok_or(Error::Unavailable)?;
            let Some(remaining) = deadline.checked_sub(now) else {
                self.expired = Some(epoch);
                return Err(Error::Expired);
            };
            if remaining == 0 {
                self.expired = Some(epoch);
                return Err(Error::Expired);
            }
            if remaining > u64::from(MAX_REMAINING_US) {
                return Err(Error::Unavailable);
            }
            self.active = Some(Arm {
                epoch,
                deadline,
                raw: raw.wrapping_add(remaining as u32),
                launched: false,
            });
            Self::program(io, 1, raw.wrapping_add(0x8000_0000))?;
            // The ONLY gate enable. Cutoff route is still off and START parked.
            io.group_on();
            Self::program(io, 0, self.active.unwrap().raw)?;
            io.cutoff_route(true);
            io.deadline_interrupt(true);
            Self::due(io, self.active.unwrap())?;
            Ok(())
        })();
        result.map_err(|error| {
            if error == Error::Expired {
                self.expired = Some(epoch);
            }
            self.fail(io, error)
        })
    }

    pub fn launch(&mut self, io: &mut impl Registers, epoch: E) -> Result<(), Error> {
        self.try_launch(io, epoch)
            .map_err(|error| self.fail(io, error))
    }

    /// The coordinator must perform error cleanup before diagnostic freeze,
    /// with both completed before facade RX restoration.
    #[cfg(feature = "sync-trace")]
    pub fn launch_before_cleanup(
        &mut self,
        io: &mut impl Registers,
        epoch: E,
    ) -> Result<(), Error> {
        self.try_launch(io, epoch)
    }

    #[cfg(feature = "sync-trace")]
    pub fn cleanup_launch_error(&mut self, io: &mut impl Registers, error: Error) -> Error {
        self.fail(io, error)
    }

    fn try_launch(&mut self, io: &mut impl Registers, epoch: E) -> Result<(), Error> {
        if self.expired == Some(epoch) {
            return Err(Error::Expired);
        }
        let arm = self.active.ok_or(Error::Unavailable)?;
        if arm.epoch != epoch || arm.launched {
            return Err(Error::HardwareFault);
        }
        let (raw, now) = Self::due(io, arm)?;
        if arm.raw.wrapping_sub(raw) <= START_LEAD_US + START_MARGIN_US
            || arm.deadline - now <= u64::from(START_LEAD_US + START_MARGIN_US)
        {
            return Err(Error::Expired);
        }
        let start = raw.wrapping_add(START_LEAD_US);
        Self::program(io, 1, start)?;
        let (after, _) = Self::due(io, arm)?;
        let lead = start.wrapping_sub(after);
        if io.event(1) || lead <= START_MARGIN_US || lead > MAX_REMAINING_US {
            return Err(Error::Expired);
        }
        self.active.as_mut().unwrap().launched = true;
        Ok(())
    }

    /// Stale IRQs without the current compare event cannot mutate the epoch.
    pub fn on_interrupt(&mut self, io: &mut impl Registers) -> bool {
        if let Some(arm) = self.active
            && io.event(0)
        {
            self.expired = Some(arm.epoch);
            io.group_off();
            io.deadline_interrupt(false);
            io.clear_event(0);
            let _ = io.event(0);
            io.clear_pending();
            return true;
        }
        io.clear_pending();
        false
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;

    struct Fake {
        raw: u32,
        now: u64,
        clock: bool,
        cc: [u32; 6],
        events: [bool; 6],
        group: bool,
        route: bool,
        interrupt: bool,
        enables: u32,
        disables: u32,
        tx: u32,
        old_latch: bool,
        jump_on_write: Option<(usize, u32)>,
        bad_readback: bool,
    }
    impl Fake {
        fn new() -> Self {
            Self {
                raw: 100,
                now: 1000,
                clock: true,
                cc: [0; 6],
                events: [false; 6],
                group: false,
                route: false,
                interrupt: false,
                enables: 0,
                disables: 0,
                tx: 0,
                old_latch: false,
                jump_on_write: None,
                bad_readback: false,
            }
        }
        fn advance(&mut self, ticks: u32) {
            let before = self.raw;
            self.raw = self.raw.wrapping_add(ticks);
            self.now += u64::from(ticks);
            // Model chronological compare events, including cutoff before START.
            let cutoff = self.cc[0].wrapping_sub(before);
            let start = self.cc[1].wrapping_sub(before);
            if start > 0
                && start <= ticks
                && self.group
                && !(self.route && cutoff > 0 && cutoff <= start)
            {
                self.tx += 1;
            }
            for index in 0..6 {
                let delta = self.cc[index].wrapping_sub(before);
                if delta > 0 && delta <= ticks {
                    self.events[index] = true;
                }
            }
            if self.events[0] && self.route {
                self.group = false;
                self.disables += 1;
            }
        }
        fn assert_off(&self) {
            assert!(!self.group);
            assert!(!self.route);
            assert!(!self.interrupt);
        }
    }
    impl Registers for Fake {
        fn pair(&mut self) -> Option<(u32, u64)> {
            self.clock.then_some((self.raw, self.now))
        }
        fn group_off(&mut self) {
            self.group = false;
        }
        fn group_on(&mut self) {
            assert!(!self.route);
            assert_eq!(self.cc[1], self.raw.wrapping_add(0x8000_0000));
            self.group = true;
            self.enables += 1;
        }
        fn group_enabled(&mut self) -> bool {
            self.group
        }
        fn cutoff_route(&mut self, enabled: bool) {
            self.route = enabled;
        }
        fn deadline_interrupt(&mut self, enabled: bool) {
            self.interrupt = enabled;
        }
        fn clear_pending(&mut self) {}
        fn event(&mut self, index: usize) -> bool {
            self.events[index]
        }
        fn clear_event(&mut self, index: usize) {
            self.events[index] = false;
        }
        fn write_cc(&mut self, index: usize, raw: u32) {
            if index == 0 {
                assert!(!self.interrupt);
                assert!(!self.route);
            }
            if self.old_latch {
                self.events[index] = true;
            }
            self.cc[index] = raw;
            if let Some((selected, ticks)) = self.jump_on_write
                && selected == index
            {
                self.jump_on_write = None;
                self.advance(ticks);
            }
        }
        fn read_cc(&mut self, index: usize) -> u32 {
            self.cc[index] ^ u32::from(self.bad_readback)
        }
        fn radio_disable(&mut self) {
            assert!(!self.group);
            self.disables += 1;
        }
    }

    #[cfg(feature = "sync-trace")]
    #[test]
    fn cutoff_copy_is_epoch_qualified_and_needs_no_register_access() {
        let mut io = Fake::new();
        let mut state = State::new();
        assert_eq!(state.raw_cutoff(1), None);
        state.arm(&mut io, 1, 1100).unwrap();
        // Getter takes no Registers argument, even with an unavailable clock.
        io.clock = false;
        assert_eq!(state.raw_cutoff(1), Some(200));
        assert_eq!(state.raw_cutoff(2), None);
        state.disarm(&mut io);
        assert_eq!(state.raw_cutoff(1), None);
        io.clock = true;
        state.arm(&mut io, 2, 1200).unwrap();
        assert_eq!(state.raw_cutoff(1), None);
        assert_eq!(state.raw_cutoff(2), Some(300));
        state.disarm(&mut io);
        assert_eq!(state.raw_cutoff(2), None);
        assert_eq!(state.arm(&mut io, 3, 1000), Err(Error::Expired));
        assert_eq!(state.raw_cutoff(3), None);
    }

    #[cfg(feature = "sync-trace")]
    #[test]
    fn diagnostic_launch_boundary_preserves_error_and_cleanup_outcome() {
        for failure in 0..5 {
            let mut io = Fake::new();
            let mut state = State::new();
            state.arm(&mut io, 1, 1100).unwrap();
            match failure {
                0 => io.clock = false,
                1 => {}
                2 => state.launch(&mut io, 1).unwrap(),
                3 => io.bad_readback = true,
                _ => io.jump_on_write = Some((1, 20)),
            }
            let expected = match failure {
                0 => Error::Unavailable,
                1..=3 => Error::HardwareFault,
                _ => Error::Expired,
            };
            let error = state
                .launch_before_cleanup(&mut io, if failure == 1 { 2 } else { 1 })
                .unwrap_err();
            assert_eq!(error, expected);
            // Original guard cleanup has priority over diagnostic freeze.
            assert_eq!(state.raw_cutoff(1), Some(200));
            io.cc[2] = 175;
            assert_eq!(state.cleanup_launch_error(&mut io, error), expected);
            assert_eq!(io.cc[2], 175); // Diagnostic sample survives State::fail.
            assert_eq!(state.raw_cutoff(1), None);
            io.assert_off();
            assert_eq!(io.enables, 1);
            assert_eq!(io.tx, u32::from(failure == 4));
            assert_eq!(state.expired(&mut io, 1), failure == 4);
        }
    }

    #[test]
    fn disarm_preserves_capture_value_and_latched_expiry_for_later_diagnostics() {
        let mut io = Fake::new();
        let mut state = State::new();
        state.arm(&mut io, 1, 1100).unwrap();
        io.cc[2] = 175;
        io.events[0] = true;
        io.events[2] = true;
        state.disarm(&mut io);
        io.assert_off();
        assert_eq!(io.cc[2], 175);
        assert!(io.events.iter().all(|event| !event));
        assert!(state.expired(&mut io, 1));
        assert!(!state.expired(&mut io, 2));
    }

    #[test]
    fn capture_source_keeps_unique_feature_gated_lease_and_address_only_radio_access() {
        let guard = include_str!("deadline_guard.rs");
        let root = include_str!("main.rs");
        let receiver = include_str!("radio_receiver.rs");
        assert!(root.contains("#[cfg(feature = \"sync-trace\")]\n            ch5: p.PPI_CH5,"));
        assert!(guard.contains(
            "#[cfg(feature = \"sync-trace\")]\n    pub(crate) ch5: Peri<'static, peripherals::PPI_CH5>"
        ));
        assert!(receiver.contains("deadline_resources: crate::deadline_guard::Resources"));
        assert!(receiver.contains("crate::deadline_guard::initialize(&radio, deadline_resources)"));
        assert!(guard.contains("radio: &Peri<'static, peripherals::RADIO>"));
        assert_eq!(guard.matches("pac::RADIO.events_end()").count(), 1);
        assert!(guard.contains("pac::RADIO.events_end().as_ptr()"));
        assert!(!guard.contains("steal("));
        assert!(!guard.contains("clone_unchecked("));
        assert!(guard.contains("Ppi::new_one_to_one(ch5, end, timer.cc(2).task_capture())"));
        assert_eq!(guard.matches("group.add_channel(").count(), 1);
        assert!(guard.contains("group.add_channel(&start)"));
        assert!(guard.contains("pac::PPI.fork(5).tep().write_value(0)"));
        assert!(guard.contains("pac::PPI.chg(group).read().ch(5)"));
        assert!(!guard.contains("chg(group).modify"));
        let prepare = guard
            .split("fn prepare_capture(")
            .nth(1)
            .unwrap()
            .split("fn arm_capture(")
            .next()
            .unwrap();
        assert!(prepare.contains("self.hardware.latest_pair"));
        assert!(!prepare.contains(".pair()"));
        assert!(!prepare.contains("SharedClock::read_static"));
        let freeze = guard.split("fn freeze_capture(").nth(1).unwrap();
        let reject = freeze.find("return self.capture.frozen()").unwrap();
        let disable = freeze.find("self.capture_off()").unwrap();
        let sample = freeze.find("self.hardware.timer.cc(2).read()").unwrap();
        let observation = freeze.find("self.hardware.pair()").unwrap();
        assert!(reject < disable && disable < sample && sample < observation);
        assert!(freeze.contains("now.checked_sub(anchor_now)?"));
        let arm = guard.split("fn arm(&self,").nth(1).unwrap();
        assert!(
            arm.find("owner.retire_current_capture()").unwrap()
                < arm.find("owner.state.arm(").unwrap()
        );
        let disarm = guard.split("fn disarm(&self)").nth(1).unwrap();
        assert!(
            disarm.find("owner.state.disarm(").unwrap()
                < disarm.find("owner.retire_current_capture()").unwrap()
        );
        let drop = guard.split("impl Drop for Owner").nth(1).unwrap();
        assert!(
            drop.find("self.state.disarm(").unwrap() < drop.find("self.capture_off()").unwrap()
        );
        let launch = guard.split("fn launch(&self,").nth(1).unwrap();
        assert!(
            launch.find("owner.arm_capture(epoch)").unwrap()
                < launch.find(".launch_before_cleanup").unwrap()
        );
        assert!(
            launch.find("owner.state.cleanup_launch_error").unwrap()
                < launch
                    .find("owner.freeze_capture(epoch.value(), false)")
                    .unwrap()
        );
        let query = guard
            .split("fn capture_end(&self,")
            .nth(1)
            .unwrap()
            .split("fn expired(&self,")
            .next()
            .unwrap();
        let query: std::string::String = query.chars().filter(|c| !c.is_whitespace()).collect();
        assert!(
            query.contains("owner.capture.frozen().filter(|f|f.snapshot.epoch==epoch.value())")
        );
        assert!(query.contains(".map(Frozen::with_completion_evidence)"));
        for forbidden in [
            "freeze_capture(",
            "capture_off(",
            ".pair()",
            "SharedClock::",
            "pac::",
            ".timer",
        ] {
            assert!(!query.contains(forbidden));
        }
        let loan = guard.split("fn with_owner<R>").nth(1).unwrap();
        assert!(
            loan.find("Exception::NonMaskableInt").unwrap()
                < loan.find("critical_section::with").unwrap()
        );
    }

    #[test]
    fn group_enabled_only_once_in_arm_and_scheduled_start_is_real() {
        let mut io = Fake::new();
        let mut state = State::new();
        state.arm(&mut io, 1, 1100).unwrap();
        assert_eq!(io.cc[0], 200);
        state.launch(&mut io, 1).unwrap();
        assert_eq!(io.cc[1], 120);
        assert_eq!(io.enables, 1);
        assert_eq!(io.tx, 0);
        io.advance(20);
        assert_eq!(io.tx, 1);
        io.advance(80);
        assert!(!io.group);
        assert!(state.expired(&mut io, 1));
        state.disarm(&mut io);
        state.disarm(&mut io);
        assert!(state.expired(&mut io, 1));
        io.assert_off();
    }

    #[test]
    fn old_compare_latched_while_masked_is_removed_after_write() {
        let mut io = Fake::new();
        io.old_latch = true;
        let mut state = State::new();
        state.arm(&mut io, 1, 1100).unwrap();
        assert!(!io.events[0]);
        assert!(!io.events[1]);
        assert!(!state.on_interrupt(&mut io));
        assert!(!state.expired(&mut io, 1));
        state.launch(&mut io, 1).unwrap();
        assert!(!io.events[1]);
    }

    #[test]
    fn crossing_target_with_route_off_requires_software_cut() {
        let mut io = Fake::new();
        io.jump_on_write = Some((0, 101));
        let mut state = State::new();
        assert_eq!(state.arm(&mut io, 7, 1100), Err(Error::Expired));
        io.assert_off();
        assert!(state.expired(&mut io, 7));
        assert_eq!(io.disables, 1);
        assert_eq!(io.tx, 0);
    }

    #[test]
    fn wrapped_raw_and_exact_remaining_bounds() {
        let mut io = Fake::new();
        io.raw = u32::MAX - 9;
        let mut state = State::new();
        state.arm(&mut io, 1, 1100).unwrap();
        assert_eq!(io.cc[0], 90);
        state.launch(&mut io, 1).unwrap();
        assert_eq!(io.cc[1], 10);
        state.disarm(&mut io);
        state
            .arm(&mut io, 2, 1000 + u64::from(MAX_REMAINING_US))
            .unwrap();
        state.disarm(&mut io);
        assert_eq!(
            state.arm(&mut io, 3, 1001 + u64::from(MAX_REMAINING_US)),
            Err(Error::Unavailable)
        );
        io.assert_off();
        assert_eq!(state.arm(&mut io, 4, 1000), Err(Error::Expired));
        assert!(state.expired(&mut io, 4));
        assert_eq!(state.arm(&mut io, 5, 999), Err(Error::Expired));
        assert!(state.expired(&mut io, 5));
        io.assert_off();
        io.clock = false;
        assert_eq!(state.arm(&mut io, 6, 1100), Err(Error::Unavailable));
        io.assert_off();
    }

    #[test]
    fn cutoff_during_payload_preparation_cannot_reenable_or_cpu_launch() {
        let mut io = Fake::new();
        let mut state = State::new();
        state.arm(&mut io, 1, 1100).unwrap();
        io.advance(100);
        assert!(!io.group);
        assert_eq!(state.launch(&mut io, 1), Err(Error::Expired));
        io.advance(100);
        assert_eq!(io.enables, 1);
        assert_eq!(io.tx, 0);
        io.assert_off();
    }

    #[test]
    fn close_start_margin_and_start_at_deadline_rejected() {
        for remaining in [20, 21, 22, 23] {
            let mut io = Fake::new();
            let mut state = State::new();
            state.arm(&mut io, 1, 1000 + remaining).unwrap();
            let result = state.launch(&mut io, 1);
            if remaining <= 22 {
                assert_eq!(result, Err(Error::Expired));
                io.assert_off();
                assert_eq!(io.tx, 0);
            } else {
                result.unwrap();
                assert_eq!(io.cc[1], 120);
            }
        }
    }

    #[test]
    fn every_launch_error_disables_routes_before_return() {
        for failure in 0..5 {
            let mut io = Fake::new();
            let mut state = State::new();
            state.arm(&mut io, 1, 1100).unwrap();
            match failure {
                0 => io.clock = false,
                1 => {}
                2 => {
                    state.launch(&mut io, 1).unwrap();
                }
                3 => io.bad_readback = true,
                _ => io.jump_on_write = Some((1, 20)),
            }
            assert!(
                state
                    .launch(&mut io, if failure == 1 { 2 } else { 1 })
                    .is_err()
            );
            io.assert_off();
            assert_eq!(io.enables, 1);
            // A real START can precede a late programming error. No DMA claim
            // is released by this helper; facade DISARM/quiescence must follow.
            assert_eq!(io.tx, u32::from(failure == 4));
        }
    }

    #[test]
    fn stale_irq_does_not_change_current_epoch_and_cutoff_is_cached() {
        let mut io = Fake::new();
        let mut state = State::new();
        state.arm(&mut io, 1, 1100).unwrap();
        io.advance(100);
        state.disarm(&mut io);
        state.arm(&mut io, 2, 1200).unwrap();
        assert!(!state.on_interrupt(&mut io));
        assert!(!state.expired(&mut io, 1));
        assert!(!state.expired(&mut io, 2));
        state.launch(&mut io, 2).unwrap();
        io.advance(100);
        assert!(state.on_interrupt(&mut io));
        assert!(state.expired(&mut io, 2));
        assert!(!state.expired(&mut io, 1));
        state.disarm(&mut io);
        assert!(state.expired(&mut io, 2));
        io.assert_off();
    }
}
