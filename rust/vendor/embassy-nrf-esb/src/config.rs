//! ESB configuration types.

use crate::error::Error;

/// TX output power.
///
/// Maps to nRF RADIO TXPOWER register values (PS §6.17.10).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TxPower {
    Neg40dBm,
    Neg20dBm,
    Neg16dBm,
    Neg12dBm,
    Neg8dBm,
    Neg4dBm,
    #[default]
    ZeroDbm,
    Pos2dBm,
    Pos3dBm,
    Pos4dBm,
    Pos5dBm,
    Pos6dBm,
    Pos7dBm,
    Pos8dBm,
}

impl TxPower {
    /// Convert to PAC TXPOWER register value.
    pub(crate) fn to_pac(self) -> crate::pac::radio::vals::Txpower {
        use crate::pac::radio::vals::Txpower;
        match self {
            Self::Neg40dBm => Txpower::NEG40_DBM,
            Self::Neg20dBm => Txpower::NEG20_DBM,
            Self::Neg16dBm => Txpower::NEG16_DBM,
            Self::Neg12dBm => Txpower::NEG12_DBM,
            Self::Neg8dBm => Txpower::NEG8_DBM,
            Self::Neg4dBm => Txpower::NEG4_DBM,
            Self::ZeroDbm => Txpower::_0_DBM,
            Self::Pos2dBm => Txpower::POS2_DBM,
            Self::Pos3dBm => Txpower::POS3_DBM,
            Self::Pos4dBm => Txpower::POS4_DBM,
            Self::Pos5dBm => Txpower::POS5_DBM,
            Self::Pos6dBm => Txpower::POS6_DBM,
            Self::Pos7dBm => Txpower::POS7_DBM,
            Self::Pos8dBm => Txpower::POS8_DBM,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Channel, CrcConfig, EFFECTIVE_RAMP_UP_US, EsbConfig, RetransmitConfig};
    use crate::error::Error;

    #[test]
    fn default_config_is_valid() {
        assert_eq!(EsbConfig::default().validate(), Ok(()));
        assert!(!EsbConfig::default().receive_auto_ack);
    }

    #[test]
    fn receive_auto_ack_policy_obeys_local_and_wire_flags() {
        for receive_auto_ack in [false, true] {
            let config = EsbConfig {
                receive_auto_ack,
                ..Default::default()
            };
            assert_eq!(
                config.validate(),
                if receive_auto_ack {
                    Err(Error::InvalidParam)
                } else {
                    Ok(())
                }
            );
            for no_ack in [false, true] {
                assert_eq!(config.should_ack(no_ack), receive_auto_ack && !no_ack);
            }
        }
    }

    #[test]
    fn payload_length_sets_max_supported_payload_size() {
        for (payload_length, expected) in [
            (0, Err(Error::InvalidParam)),
            (253, Err(Error::InvalidParam)),
            (252, Ok(())),
        ] {
            let config = EsbConfig {
                payload_length,
                ..Default::default()
            };
            assert_eq!(config.validate(), expected);
        }
    }

    #[test]
    fn payload_length_builder_sets_max_payload_size() {
        let config = EsbConfig::default().with_payload_length(64);

        assert_eq!(config.payload_length, 64);
        assert_eq!(config.validate(), Ok(()));
    }

    #[test]
    fn ack_timeout_has_minimum() {
        let config = EsbConfig {
            ack_timeout_us: 43,
            ..Default::default()
        };
        assert_eq!(config.validate(), Err(Error::InvalidParam));
        let config = EsbConfig {
            ack_timeout_us: 44,
            retransmit: RetransmitConfig {
                delay_us: 141,
                ..Default::default()
            },
            ..Default::default()
        };
        assert_eq!(config.validate(), Ok(()));
    }

    #[test]
    fn retransmit_delay_must_exceed_ack_timeout_plus_guard_and_ramp_up() {
        let mut config = EsbConfig {
            ack_timeout_us: 120,
            retransmit: RetransmitConfig {
                count: 3,
                delay_us: 182,
            },
            ..Default::default()
        };

        assert_eq!(config.validate(), Err(Error::InvalidParam));

        config.retransmit.delay_us = 183;
        assert_eq!(config.validate(), Ok(()));

        config.ack_timeout_us = 44;
        let minimum_delay = (config.ack_timeout_us + 62).max(EFFECTIVE_RAMP_UP_US);
        config.retransmit.delay_us = minimum_delay;
        assert_eq!(config.validate(), Err(Error::InvalidParam));

        config.retransmit.delay_us = minimum_delay + 1;
        assert_eq!(config.validate(), Ok(()));
    }

    #[test]
    fn ramp_mode_selects_ptx_timer_values() {
        let config = EsbConfig::default();
        #[cfg(feature = "fast-ru")]
        {
            assert_eq!(EFFECTIVE_RAMP_UP_US, 40);
            assert_eq!(config.retransmit_timer_delay_us(), 460);
            assert_eq!(config.ack_timer_timeout_us(), 160);
        }
        #[cfg(not(feature = "fast-ru"))]
        {
            assert_eq!(EFFECTIVE_RAMP_UP_US, 140);
            assert_eq!(config.retransmit_timer_delay_us(), 360);
            assert_eq!(config.ack_timer_timeout_us(), 260);
        }
    }

    #[test]
    fn ptx_timer_values_saturate_at_u16_bounds() {
        let config = EsbConfig {
            retransmit: RetransmitConfig {
                delay_us: 0,
                ..Default::default()
            },
            ack_timeout_us: u16::MAX,
            ..Default::default()
        };
        assert_eq!(config.retransmit_timer_delay_us(), 0);
        assert_eq!(config.ack_timer_timeout_us(), u16::MAX);
    }

    #[test]
    fn channel_retransmit_count_and_crc_length_are_bounded() {
        let config = EsbConfig {
            channel: Channel(101),
            ..Default::default()
        };
        assert_eq!(config.validate(), Err(Error::InvalidParam));
        let config = EsbConfig {
            retransmit: RetransmitConfig {
                count: 16,
                ..Default::default()
            },
            ..Default::default()
        };
        assert_eq!(config.validate(), Err(Error::InvalidParam));
        let config = EsbConfig {
            crc: CrcConfig {
                length: 3,
                ..Default::default()
            },
            ..Default::default()
        };
        assert_eq!(config.validate(), Err(Error::InvalidParam));
    }
}

/// Radio ramp-up time in microseconds (normal mode).
pub const RAMP_UP_US: u16 = 140;
/// Radio ramp-up time in microseconds (fast ramp-up mode).
pub const RAMP_UP_FAST_US: u16 = 40;

/// Ramp-up time matching the RADIO MODECNF0.RU feature selection.
#[cfg(feature = "fast-ru")]
pub(crate) const EFFECTIVE_RAMP_UP_US: u16 = RAMP_UP_FAST_US;
#[cfg(not(feature = "fast-ru"))]
pub(crate) const EFFECTIVE_RAMP_UP_US: u16 = RAMP_UP_US;

/// ESB protocol bitrate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Bitrate {
    /// 1 Mbps (longer range, lower current).
    Mbps1,
    /// 2 Mbps (shorter range, higher throughput) — default for ESB.
    #[default]
    Mbps2,
}

/// ESB RF channel (0–100, maps to 2400–2500 MHz).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Channel(pub u8);

impl Channel {
    /// Minimum channel value.
    pub const MIN: u8 = 0;
    /// Maximum channel value.
    pub const MAX: u8 = 100;
}

impl Default for Channel {
    fn default() -> Self {
        Self(2)
    }
}

/// CRC configuration.
///
/// For standard ESB CRC16, the full polynomial is 0x11021. Configuration
/// stores its lower 16 bits; the register layer adds the degree bit when
/// writing the 24-bit CRCPOLY register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CrcConfig {
    /// CRC length in bytes: 0 (off), 1, or 2.
    pub length: u8,
    /// Initial CRC value.
    pub init: u16,
    /// Polynomial (lower bits; the register layer adds the degree bit).
    pub poly: u16,
}

impl Default for CrcConfig {
    fn default() -> Self {
        Self {
            length: 2,
            init: 0xFFFF,
            poly: 0x1021,
        }
    }
}

/// Retransmission configuration (PTX only).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RetransmitConfig {
    /// Maximum retransmit attempts (0–15).
    pub count: u8,
    /// Delay in microseconds between retransmits.
    ///
    /// Must be > `ack_timeout` + 62 µs. Typical: 500 µs.
    pub delay_us: u16,
}

impl Default for RetransmitConfig {
    fn default() -> Self {
        Self {
            count: 3,
            delay_us: 500,
        }
    }
}

/// Top-level ESB configuration.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EsbConfig {
    /// Bitrate (1 or 2 Mbps).
    pub bitrate: Bitrate,
    /// Automatically acknowledge ACK-request packets in PRX mode.
    /// Only false is supported by this fork (and is the default). Valid new
    /// ACK-request packets may be delivered, but never cause ACK TX.
    pub receive_auto_ack: bool,
    /// RF channel (0–100).
    pub channel: Channel,
    /// CRC configuration.
    pub crc: CrcConfig,
    /// Retransmit configuration (PTX only).
    pub retransmit: RetransmitConfig,
    /// ACK timeout in microseconds (PTX waits this long for ACK).
    ///
    /// Must be >= 44 µs. Default: 120 µs.
    pub ack_timeout_us: u16,
    /// Maximum payload length accepted by software and RADIO EasyDMA (1–252 bytes).
    ///
    /// ESB dynamic payloads shorter than this value are still allowed. Packets
    /// longer than this value are rejected by `send()` / `send_no_ack()` /
    /// `send_ack_payload()` and RADIO `PCNF1.MAXLEN`.
    ///
    /// Higher-level split protocols should set this to at least their framed
    /// packet length. For `transport` frames, use
    /// `transport::required_esb_payload_len(app_payload_len)` to calculate the
    /// required value before constructing the driver.
    pub payload_length: u8,
    /// TX output power.
    pub tx_power: TxPower,
}

impl Default for EsbConfig {
    fn default() -> Self {
        Self {
            bitrate: Bitrate::default(),
            receive_auto_ack: false,
            channel: Channel::default(),
            crc: CrcConfig::default(),
            retransmit: RetransmitConfig::default(),
            ack_timeout_us: 120,
            payload_length: 32,
            tx_power: TxPower::default(),
        }
    }
}

impl EsbConfig {
    pub(crate) fn retransmit_timer_delay_us(&self) -> u16 {
        self.retransmit
            .delay_us
            .saturating_sub(EFFECTIVE_RAMP_UP_US)
    }

    pub(crate) fn ack_timer_timeout_us(&self) -> u16 {
        self.ack_timeout_us.saturating_add(EFFECTIVE_RAMP_UP_US)
    }

    pub(crate) fn should_ack(&self, no_ack: bool) -> bool {
        self.receive_auto_ack && !no_ack
    }

    /// Return a copy of this config with a different maximum payload length.
    ///
    /// This is a convenience for higher-level protocols that need payloads
    /// larger than the ESB-compatible 32-byte default. The value is still
    /// checked by [`validate`](Self::validate); valid lengths are 1–252 bytes.
    pub const fn with_payload_length(mut self, payload_length: u8) -> Self {
        self.payload_length = payload_length;
        self
    }

    /// Validate configuration against Nordic ESB constraints.
    ///
    /// Returns `Ok(())` if valid, `Err(Error::InvalidParam)` otherwise.
    pub fn validate(&self) -> Result<(), Error> {
        if self.receive_auto_ack {
            return Err(Error::InvalidParam);
        }
        // Payload length
        if self.payload_length > 252 || self.payload_length == 0 {
            return Err(Error::InvalidParam);
        }

        // ACK timeout minimum
        if self.ack_timeout_us < 44 {
            return Err(Error::InvalidParam);
        }

        // Retransmit delay must be > ack_timeout + 62 µs
        // AND > RAMP_UP_TIME (radio needs time to re-enable)
        let min_retransmit_delay = self.ack_timeout_us.saturating_add(62);
        if self.retransmit.delay_us <= min_retransmit_delay
            || self.retransmit.delay_us <= EFFECTIVE_RAMP_UP_US
        {
            return Err(Error::InvalidParam);
        }

        // Channel range
        if self.channel.0 > Channel::MAX {
            return Err(Error::InvalidParam);
        }

        // Retransmit count range
        if self.retransmit.count > 15 {
            return Err(Error::InvalidParam);
        }

        // CRC length: 0, 1, or 2
        if self.crc.length > 2 {
            return Err(Error::InvalidParam);
        }

        Ok(())
    }
}
