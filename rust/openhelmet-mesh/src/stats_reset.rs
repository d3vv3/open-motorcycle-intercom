//! Stateless reset construction. Queue storage is deliberately not an input.
//! The adapter supplies a new sampled epoch and current queue depths.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Reset {
    pub epoch: u32,
    pub latency_min: u32,
    pub tx_high_water: u32,
    pub jitter_depth: u8,
    pub control_depth: u8,
    pub control_high_water: u8,
}

pub const fn construct(epoch: u32, jitter: u8, control: u8, tx: u32) -> Reset {
    Reset {
        epoch,
        latency_min: u32::MAX,
        tx_high_water: tx,
        jitter_depth: jitter,
        control_depth: control,
        control_high_water: control,
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn supplied_epoch_and_retained_depth_seeds() {
        let reset = super::construct(0, 4, 7, 19);
        assert_eq!(reset.epoch, 0); // zero is a valid sampled epoch
        assert_eq!(reset.latency_min, u32::MAX);
        assert_eq!(reset.jitter_depth, 4);
        assert_eq!(reset.control_depth, 7);
        assert_eq!(reset.control_high_water, 7);
        assert_eq!(reset.tx_high_water, 19);
    }
}
