//! Diagnostic observations only. All counters wrap; no scheduling decisions.
use crate::bridge_rpc::RequestKind;
use core::sync::atomic::{AtomicU8, Ordering};

#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum Phase {
    Commands,
    CommandWork,
    Timers,
    RadioService,
    RxDequeue,
    RxWork,
    Ingress,
    ServiceTiming,
    Wait,
}
pub const PHASES: usize = 9;
pub const RPC_BUCKETS: usize = 5;
pub static ACTOR_PHASE: AtomicU8 = AtomicU8::new(Phase::Wait as u8);
pub static RPC_IN_FLIGHT: AtomicU8 = AtomicU8::new(0);

pub fn phase(value: Phase) {
    ACTOR_PHASE.store(value as u8, Ordering::Relaxed);
}
pub fn phase_bucket(raw: u8) -> usize {
    usize::from(raw).min(PHASES - 1)
}
/// none, bridge_deliver_rx, status_event_log, command, other.
pub fn rpc_bucket(raw: u8) -> usize {
    match raw {
        0 => 0,
        x if x == RequestKind::AudioBundle as u8 => 1,
        x if x == RequestKind::Status as u8
            || x == RequestKind::Event as u8
            || x == RequestKind::Log as u8 =>
        {
            2
        }
        x if x == RequestKind::Ack as u8
            || x == RequestKind::DiscardAudio as u8
            || x == RequestKind::Delivery as u8 =>
        {
            3
        }
        _ => 4,
    }
}
pub struct RpcGuard;
impl RpcGuard {
    pub fn new(kind: RequestKind) -> Self {
        RPC_IN_FLIGHT.store(kind as u8, Ordering::Relaxed);
        Self
    }
}
impl Drop for RpcGuard {
    fn drop(&mut self) {
        RPC_IN_FLIGHT.store(0, Ordering::Relaxed);
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Histogram {
    pub bins: [u32; 7],
    pub max: u32,
    pub count: u32,
    /// Wrapping total of recorded µs; sum/count gives the mean the bins hide.
    pub sum: u32,
}
impl Histogram {
    pub fn record(&mut self, us: u32) {
        let bucket = [100, 200, 400, 600, 800, 1000]
            .iter()
            .position(|limit| us < *limit)
            .unwrap_or(6);
        self.bins[bucket] = self.bins[bucket].wrapping_add(1);
        self.count = self.count.wrapping_add(1);
        self.max = self.max.max(us);
        self.sum = self.sum.wrapping_add(us);
    }
    pub fn elapsed(&mut self, now: u32, start: u32) {
        self.record(now.wrapping_sub(start));
    }
}
#[cfg(feature = "pipeline-trace")]
pub(crate) struct Csv<'a>(pub &'a [u32]);
#[cfg(feature = "pipeline-trace")]
impl core::fmt::Display for Csv<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for (i, value) in self.0.iter().enumerate() {
            if i != 0 {
                f.write_str(",")?;
            }
            core::fmt::Display::fmt(value, f)?;
        }
        Ok(())
    }
}
#[cfg(feature = "pipeline-trace")]
impl core::fmt::Display for Histogram {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "{},{},{},{}",
            Csv(&self.bins),
            self.max,
            self.count,
            self.sum
        )
    }
}
#[derive(Clone, Copy, Debug, Default)]
pub struct Stats {
    pub slot_alarm_phase: [u32; PHASES],
    pub frame_alarm_phase: [u32; PHASES],
    pub slot_alarm_rpc: [u32; RPC_BUCKETS],
    pub frame_alarm_rpc: [u32; RPC_BUCKETS],
    pub entry0: Histogram,
    pub entryn: Histogram,
    pub irq_service: Histogram,
    /// Frame-alarm drains only: accept → frame_timer returned / → drain returned.
    pub frame_dispatch: Histogram,
    pub frame_drain: Histogram,
    /// Frame/Slot alarms serviced >= LATE_SERVICE_US after their IRQ, by the
    /// actor phase / bridge RPC that was active at IRQ time.
    pub late_alarm_phase: [u32; PHASES],
    pub late_alarm_rpc: [u32; RPC_BUCKETS],
    /// Radio budget rejects: shortfall (now + airtime budget - deadline) in µs,
    /// and the rejected packet's wire length class: 65 B (current only),
    /// 113 B (one history frame), other.
    pub budget_short: Histogram,
    pub budget_reject_len: [u32; 3],
}
pub const LATE_SERVICE_US: u32 = 400;
impl Stats {
    pub const ZERO: Self = Self {
        slot_alarm_phase: [0; PHASES],
        frame_alarm_phase: [0; PHASES],
        slot_alarm_rpc: [0; RPC_BUCKETS],
        frame_alarm_rpc: [0; RPC_BUCKETS],
        entry0: Histogram {
            bins: [0; 7],
            max: 0,
            count: 0,
            sum: 0,
        },
        entryn: Histogram {
            bins: [0; 7],
            max: 0,
            count: 0,
            sum: 0,
        },
        irq_service: Histogram {
            bins: [0; 7],
            max: 0,
            count: 0,
            sum: 0,
        },
        frame_dispatch: Histogram {
            bins: [0; 7],
            max: 0,
            count: 0,
            sum: 0,
        },
        frame_drain: Histogram {
            bins: [0; 7],
            max: 0,
            count: 0,
            sum: 0,
        },
        late_alarm_phase: [0; PHASES],
        late_alarm_rpc: [0; RPC_BUCKETS],
        budget_short: Histogram {
            bins: [0; 7],
            max: 0,
            count: 0,
            sum: 0,
        },
        budget_reject_len: [0; 3],
    };
    pub fn budget_reject(&mut self, shortfall_us: u32, packet_len: usize) {
        self.budget_short.record(shortfall_us);
        let i = match packet_len {
            65 => 0,
            113 => 1,
            _ => 2,
        };
        self.budget_reject_len[i] = self.budget_reject_len[i].wrapping_add(1);
    }
    pub fn late_service(&mut self, elapsed_us: u32, phase: u8, rpc: u8) {
        if elapsed_us >= LATE_SERVICE_US {
            let p = &mut self.late_alarm_phase[phase_bucket(phase)];
            *p = p.wrapping_add(1);
            let r = &mut self.late_alarm_rpc[rpc_bucket(rpc)];
            *r = r.wrapping_add(1);
        }
    }
    pub fn alarm(&mut self, frame: bool, phase: u8, rpc: u8) {
        let (phases, rpcs) = if frame {
            (&mut self.frame_alarm_phase, &mut self.frame_alarm_rpc)
        } else {
            (&mut self.slot_alarm_phase, &mut self.slot_alarm_rpc)
        };
        let p = &mut phases[phase_bucket(phase)];
        *p = p.wrapping_add(1);
        let r = &mut rpcs[rpc_bucket(rpc)];
        *r = r.wrapping_add(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn marker_buckets() {
        use crate::bridge_rpc::Request;
        assert_eq!(
            Request::Delivery(true).kind() as u8,
            RequestKind::Delivery as u8
        );
        assert_eq!(
            Request::DiscardAudio.kind() as u8,
            RequestKind::DiscardAudio as u8
        );
        assert_eq!(
            Request::log(b"trace").unwrap().kind() as u8,
            RequestKind::Log as u8
        );
        assert_eq!(
            Request::event(1, &[]).unwrap().kind() as u8,
            RequestKind::Event as u8
        );
        {
            let _guard = RpcGuard::new(RequestKind::AudioBundle);
            assert_eq!(RPC_IN_FLIGHT.load(Ordering::Relaxed), 5);
        }
        assert_eq!(RPC_IN_FLIGHT.load(Ordering::Relaxed), 0);
        phase(Phase::Ingress);
        assert_eq!(ACTOR_PHASE.load(Ordering::Relaxed), Phase::Ingress as u8);
        for (i, p) in [
            Phase::Commands,
            Phase::CommandWork,
            Phase::Timers,
            Phase::RadioService,
            Phase::RxDequeue,
            Phase::RxWork,
            Phase::Ingress,
            Phase::ServiceTiming,
            Phase::Wait,
        ]
        .into_iter()
        .enumerate()
        {
            assert_eq!(phase_bucket(p as u8), i);
        }
        for (raw, bucket) in [
            (0, 0),
            (1, 2),
            (2, 3),
            (3, 2),
            (4, 2),
            (5, 1),
            (6, 3),
            (7, 3),
            (255, 4),
        ] {
            assert_eq!(rpc_bucket(raw), bucket);
        }
        let mut s = Stats::ZERO;
        s.slot_alarm_phase[0] = u32::MAX;
        s.slot_alarm_rpc[0] = u32::MAX;
        s.alarm(false, 0, 0);
        s.alarm(true, 8, 5);
        assert_eq!(s.slot_alarm_phase[0], 0);
        assert_eq!(s.slot_alarm_rpc[0], 0);
        assert_eq!(s.frame_alarm_phase[8], 1);
        assert_eq!(s.frame_alarm_rpc[1], 1);
        s.late_service(LATE_SERVICE_US - 1, 8, 5);
        assert_eq!(s.late_alarm_phase, [0; PHASES]);
        s.late_service(LATE_SERVICE_US, 8, 5);
        s.late_service(u32::MAX, 255, 255);
        assert_eq!(s.late_alarm_phase[8], 2);
        assert_eq!(s.late_alarm_rpc[1], 1);
        assert_eq!(s.late_alarm_rpc[4], 1);
        s.budget_reject(150, 65);
        s.budget_reject(450, 113);
        s.budget_reject(0, 161);
        assert_eq!(s.budget_reject_len, [1, 1, 1]);
        assert_eq!(s.budget_short.bins, [1, 1, 0, 1, 0, 0, 0]);
        assert_eq!((s.budget_short.max, s.budget_short.count), (450, 3));
    }
    #[test]
    fn histogram_boundaries_and_wrap() {
        let mut h = Histogram::default();
        for us in [99, 100, 199, 200, 399, 400, 599, 600, 799, 800, 999, 1000] {
            h.record(us);
        }
        assert_eq!(h.bins, [1, 2, 2, 2, 2, 2, 1]);
        assert_eq!((h.max, h.count), (1000, 12));
        h.count = u32::MAX;
        h.bins[0] = u32::MAX;
        h.elapsed(20, u32::MAX - 29);
        assert_eq!((h.bins[0], h.count, h.max), (0, 0, 1000));
    }
}
