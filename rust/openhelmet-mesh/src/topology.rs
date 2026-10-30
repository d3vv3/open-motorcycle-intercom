//! Task-owned topology observations, production cadence and synchronous wire sequencing.

#[cfg(test)]
mod audio_tests {
    #[test]
    fn reordered_audio_cannot_inflate_received_or_loss_and_clear_restarts() {
        let mut t = super::Topology::default();
        for seq in [100, 99, 99, 100, 101] {
            t.audio(2, seq);
        }
        assert_eq!((t.neighbors[1].received, t.neighbors[1].lost), (2, 0));
        t.clear(2);
        t.audio(2, 0);
        t.audio(2, 1);
        assert_eq!((t.neighbors[1].received, t.neighbors[1].lost), (2, 0));
    }
}
use crate::{
    adaptive as a,
    core::{SlotMap, node_bit},
};
#[derive(Clone, Copy, Default)]
struct Neighbor {
    seen: u32,
    direct: u32,
    report_ms: u32,
    report_seq: u32,
    rssi: i8,
    quality: u8,
    report_seen: bool,
    audio_seq: crate::core::Seq8,
    received: u32,
    lost: u32,
}
#[derive(Default)]
pub struct Topology {
    neighbors: [Neighbor; 8],
    // Bytes keep the enclosing opaque Owner aligned to four, including on ARM.
    last_loop: [u8; 8],
    production: u32,
    wire: u32,
    pending: u32,
    pending_session: u32,
    pending_term: u32,
    pending_leader: u8,
    pending_local: u8,
    pending_identity: a::Identity,
}
/// Receive outcome distinguishes independent direct evidence from graph commit.
#[derive(Clone, Copy, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum ReceiveOutcome {
    #[default]
    Rejected = 0,
    Observed = 1,
    Committed = 2,
}
impl Topology {
    /// Route binding and envelope are checked against the live Owner by the
    /// adapter. Keep estimator observation before graph sequence admission:
    /// a direct duplicate of a relayed report still supplies direct evidence.
    #[allow(clippy::too_many_arguments)]
    pub fn receive(
        &mut self,
        s: &mut a::Adaptive,
        source: u8,
        mac: [u8; 6],
        report: &a::Topology,
        relayed: bool,
        rssi: i16,
        received_ms: u32,
        now: u32,
    ) -> ReceiveOutcome {
        if report.origin.address_len != 6
            || !a::control_authorized(Some(s), 14, source)
            || !a::report_valid(Some(s), source, Some(report))
        {
            return ReceiveOutcome::Rejected;
        }
        let observed = self.observe(s, source, mac, relayed, rssi, received_ms);
        if observed {
            self.direct_report(source, report.report_seq, received_ms);
        }
        if a::report(Some(s), source, Some(report), relayed, now) {
            ReceiveOutcome::Committed
        } else if observed {
            ReceiveOutcome::Observed
        } else {
            ReceiveOutcome::Rejected
        }
    }
    pub fn cadence(&self) -> [u8; 8] {
        self.last_loop
    }
    pub fn restore_cadence(&mut self, cadence: [u8; 8]) {
        self.last_loop = cadence;
    }
    /// Adaptive/session init does not restart the mesh-task lifetime cadence.
    pub fn reinit(&mut self) {
        self.neighbors = [Neighbor::default(); 8];
        self.production = 0;
        self.wire = 0;
        self.pending = 0;
    }
    pub fn clear(&mut self, id: u8) {
        if let Some(n) = id
            .checked_sub(1)
            .and_then(|i| self.neighbors.get_mut(i as usize))
        {
            *n = Neighbor::default();
        }
    }
    pub fn due(&mut self, now: i64) -> bool {
        if now.wrapping_sub(i64::from_le_bytes(self.last_loop)) < i64::from(a::REPORT_MS) * 1000 {
            return false;
        }
        self.last_loop = now.to_le_bytes();
        true
    }
    pub fn observe(
        &mut self,
        s: &a::Adaptive,
        id: u8,
        mac: [u8; 6],
        relayed: bool,
        rssi: i16,
        now: u32,
    ) -> bool {
        let identity = s.member_identity(id);
        if node_bit(id) == 0
            || id == s.session().0
            || relayed
            || !a::identity_valid(Some(&identity))
            || identity.address_len != 6
            || identity.address != mac
        {
            return false;
        }
        let n = &mut self.neighbors[(id - 1) as usize];
        let rssi = rssi.clamp(-127, 0) as i8;
        n.rssi = if n.direct == 0 {
            rssi
        } else {
            ((3 * i16::from(n.rssi) + i16::from(rssi)) / 4) as i8
        };
        n.seen = now;
        n.direct = now;
        true
    }
    /// Called only after report validation and authenticated direct observation.
    pub fn direct_report(&mut self, id: u8, seq: u32, now: u32) {
        let Some(n) = id
            .checked_sub(1)
            .and_then(|i| self.neighbors.get_mut(i as usize))
        else {
            return;
        };
        let delta = seq.wrapping_sub(n.report_seq);
        if n.report_seen && (delta == 0 || delta >= 0x8000_0000) {
            return;
        }
        let sample = if n.report_seen {
            (100 / delta) as u8
        } else {
            100
        };
        n.quality = if n.report_seen {
            ((3 * u16::from(n.quality) + u16::from(sample) + 2) / 4) as u8
        } else {
            sample
        };
        n.report_seen = true;
        n.report_seq = seq;
        n.report_ms = now;
    }
    pub fn audio(&mut self, id: u8, seq: u8) {
        let Some(n) = id
            .checked_sub(1)
            .and_then(|i| self.neighbors.get_mut(i as usize))
        else {
            return;
        };
        let result = crate::core::seq8_accept(Some(&mut n.audio_seq), seq);
        if result.classification == crate::core::SeqClass::Gap {
            n.lost = n.lost.wrapping_add(u32::from(result.gap));
        }
        if matches!(
            result.classification,
            crate::core::SeqClass::First
                | crate::core::SeqClass::InOrder
                | crate::core::SeqClass::Gap
        ) {
            n.received = n.received.wrapping_add(1);
        }
    }
    pub fn relay_seen(&mut self, id: u8, now: u32) {
        if let Some(n) = id
            .checked_sub(1)
            .and_then(|i| self.neighbors.get_mut(i as usize))
        {
            n.seen = now;
        }
    }
    pub fn fresh(&self, id: u8, now: u32, require_nonzero: bool) -> bool {
        id.checked_sub(1)
            .and_then(|i| self.neighbors.get(i as usize))
            .is_some_and(|n| {
                (!require_nonzero || n.seen != 0) && now.wrapping_sub(n.seen) <= a::EXPIRE_MS
            })
    }
    pub fn produce(
        &mut self,
        s: &mut a::Adaptive,
        active: bool,
        map: SlotMap,
        now: u32,
    ) -> Option<a::Topology> {
        let (local, leader, members, term) = s.session();
        if !active || local == 0 {
            return None;
        }
        let mut r = a::Topology {
            origin: s.member_identity(local),
            report_seq: self.production,
            leader_id: leader,
            term,
            slot_map: map,
            ..Default::default()
        };
        self.production = self.production.wrapping_add(1);
        for (i, n) in self.neighbors.iter().enumerate() {
            if members & node_bit(i as u8 + 1) == 0
                || n.direct == 0
                || !n.report_seen
                || now.wrapping_sub(n.direct) > a::EXPIRE_MS
                || now.wrapping_sub(n.report_ms) > a::EXPIRE_MS
            {
                continue;
            }
            r.direct_mask |= node_bit(i as u8 + 1);
            r.rssi_dbm[i] = n.rssi;
            r.quality[i] = n.quality;
        }
        a::report(Some(s), local, Some(&r), false, now).then_some(r)
    }
    /// Token is supplied by the adapter's lifecycle-independent allocator.
    pub fn stamp(
        &mut self,
        s: &a::Adaptive,
        mut r: a::Topology,
        token: u32,
    ) -> Option<(a::Topology, u32)> {
        if r.term != s.session().3 || r.leader_id != s.session().1 || token == 0 {
            return None;
        }
        r.report_seq = self.wire;
        self.wire = self.wire.wrapping_add(1);
        self.pending = token;
        self.pending_session = token;
        self.pending_term = s.session().3;
        self.pending_leader = s.session().1;
        self.pending_local = s.session().0;
        self.pending_identity = s.member_identity(self.pending_local);
        Some((r, token))
    }
    pub fn result_current(&mut self, s: &a::Adaptive, token: u32, success: bool) -> bool {
        if self.pending_term != s.session().3
            || self.pending_leader != s.session().1
            || self.pending_local != s.session().0
            || self.pending_identity != s.member_identity(self.pending_local)
        {
            return false;
        }
        self.result(token, token, success)
    }
    pub fn result(&mut self, token: u32, context: u32, success: bool) -> bool {
        if token == 0 || self.pending != token || self.pending_session != context {
            return false;
        }
        self.pending = 0;
        if !success {
            self.wire = self.wire.wrapping_sub(1);
        }
        true
    }
}
