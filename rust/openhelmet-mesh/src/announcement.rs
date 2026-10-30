//! Local request advertisement bookkeeping; enqueue execution remains outside the owner.
use crate::adaptive::SpeakerRequest;

#[derive(Clone, Copy)]
pub struct Prepared {
    pub token: u32,
    pub request: SpeakerRequest,
    pub now: u32,
}
#[derive(Default)]
pub struct Announcement {
    sequence: u32,
    announced: bool,
    last_ms: u32,
    term: u32,
    pending: Option<Prepared>,
}
impl Announcement {
    /// Handover preserves sequence but forces the next active advertisement.
    pub fn handover(&mut self) {
        self.pending = None;
        self.term = 0;
    }
    pub fn prepare(
        &mut self,
        term: u32,
        active: bool,
        granted: bool,
        now: u32,
        frame: u32,
        token: u32,
    ) -> Option<Prepared> {
        self.pending = None;
        if !active && !self.announced {
            self.term = term;
            return None;
        }
        let interval = if granted { 1000 } else { 400 };
        if active == self.announced
            && self.term == term
            && (!active || now.wrapping_sub(self.last_ms) < interval)
        {
            return None;
        }
        self.sequence = self.sequence.wrapping_add(1);
        let p = Prepared {
            token,
            request: SpeakerRequest {
                term,
                request_seq: self.sequence,
                frame_counter: frame,
                active: u8::from(active),
            },
            now,
        };
        self.pending = Some(p);
        Some(p)
    }
    pub fn complete(&mut self, token: u32, term: u32, success: bool) -> Option<Prepared> {
        let p = self.pending?;
        if p.token != token {
            return None;
        }
        self.pending = None;
        if !success || p.request.term != term {
            return None;
        }
        self.announced = p.request.active != 0;
        self.last_ms = p.now;
        self.term = p.request.term;
        Some(p)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn cadence_failure_release_handover_and_wrap() {
        let mut a = Announcement::default();
        assert!(a.prepare(7, false, false, 0, 0, 1).is_none());
        let p = a
            .prepare(7, true, false, u32::MAX - 100, u32::MAX, 2)
            .unwrap();
        assert_eq!(p.request.request_seq, 1);
        assert!(a.complete(2, 7, false).is_none());
        let p = a.prepare(7, true, false, u32::MAX - 100, 0, 3).unwrap();
        assert_eq!(p.request.request_seq, 2);
        assert!(a.complete(3, 7, true).is_some());
        assert!(a.complete(3, 7, true).is_none());
        assert!(a.prepare(7, true, false, 298, 0, 4).is_none());
        assert!(a.prepare(7, true, false, 299, 0, 5).is_some());
        assert!(a.complete(5, 7, true).is_some());
        assert!(a.prepare(7, true, true, 1298, 0, 6).is_none());
        assert!(a.prepare(7, true, true, 1299, 0, 7).is_some());
        a.handover();
        assert!(a.complete(7, 7, true).is_none());
        assert!(a.prepare(8, true, true, 1300, 0, 8).is_some());
        a.complete(8, 8, true).unwrap();
        let p = a.prepare(8, false, true, 1301, 0, 9).unwrap();
        assert_eq!(p.request.active, 0);
        a.complete(9, 8, true).unwrap();
        assert!(a.prepare(9, false, false, 1302, 0, 10).is_none());
        a.sequence = u32::MAX;
        assert_eq!(
            a.prepare(9, true, false, 1303, 0, 11)
                .unwrap()
                .request
                .request_seq,
            0
        );
    }
}
