//! Safe, allocation-free counterparts of all 23 `mesh_core_*` functions.
//! Function names omit the C prefix; nullable arguments use `Option`.

pub const MAX_NODES: usize = 8;
pub const MAX_ACTIVE_SPEAKERS: usize = 2;
pub const DEDUPE_CAPACITY: usize = 32;
pub const CURRENT_ACTIVE: u8 = 0x01;
pub const PREVIOUS1_PRESENT: u8 = 0x02;
pub const PREVIOUS1_ACTIVE: u8 = 0x04;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PacketKey {
    pub packet_type: u8,
    pub src_id: u8,
    pub seq: u8,
    pub valid: bool,
}

/// Cursor cannot be forged: only reset and successful accept advance it.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Dedupe {
    entries: [PacketKey; DEDUPE_CAPACITY],
    next: u8,
    audio: [AudioReplay; MAX_NODES],
}

// Byte arrays preserve the opaque C owner's byte alignment and zero-init contract.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct AudioReplay {
    seq: [u8; 2],
    admitted_ms: [u8; 4],
    initialized: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DedupeSnapshot {
    pub entries: [PacketKey; DEDUPE_CAPACITY],
    pub next: u8,
}

impl Dedupe {
    pub fn snapshot(&self) -> DedupeSnapshot {
        DedupeSnapshot {
            entries: self.entries,
            next: self.next,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum SeqClass {
    First,
    InOrder,
    Duplicate,
    Gap,
    OldReset,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SeqResult {
    pub classification: SeqClass,
    pub gap: u16,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Seq8 {
    pub last: u8,
    pub initialized: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Seq16 {
    pub last: u16,
    pub initialized: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PeerSnapshot {
    pub node_id: u8,
    pub heard_bitmap: u8,
    pub active: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct JoinAssignment {
    pub assigned_id: u8,
    pub slot_index: u8,
    pub coordinator_id: u8,
}

/// Semantic protocol fields, not a packed wire representation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SlotMap {
    pub slot_count: u8,
    pub slot_ids: [u8; MAX_NODES],
    pub active_speaker_count: u8,
    pub active_speaker_ids: [u8; MAX_ACTIVE_SPEAKERS],
    pub relay_masks: [u8; MAX_ACTIVE_SPEAKERS],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SlotMapResult {
    pub local_slot: i8,
    pub member_bitmap: u8,
    pub member_count: u8,
}

/// Borrowed semantic view. Slice lengths replace C pointer/length pairs.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AudioBundleView<'a> {
    pub previous1_data: &'a [u8],
    pub previous2_data: &'a [u8],
    pub current_data: &'a [u8],
    pub current_seq: u16,
    pub stream_id: u8,
    pub flags: u8,
    pub codec: u8,
}

pub fn node_id_valid(node_id: u8) -> bool {
    node_id > 0 && usize::from(node_id) <= MAX_NODES
}

pub fn node_bit(node_id: u8) -> u8 {
    if node_id_valid(node_id) {
        1 << (node_id - 1)
    } else {
        0
    }
}

pub fn bitmap_contains(bitmap: u8, node_id: u8) -> bool {
    let bit = node_bit(node_id);
    bit != 0 && bitmap & bit != 0
}

pub fn first_free_node_id(occupied_bitmap: u8) -> u8 {
    (1..=8)
        .find(|&id| !bitmap_contains(occupied_bitmap, id))
        .unwrap_or(0)
}

pub fn slot_for_node_id(node_id: u8) -> i8 {
    if node_id_valid(node_id) {
        (node_id - 1) as i8
    } else {
        -1
    }
}

pub fn dedupe_reset(cache: Option<&mut Dedupe>) {
    if let Some(cache) = cache {
        *cache = Dedupe::default();
    }
}

pub fn dedupe_contains(cache: Option<&Dedupe>, packet_type: u8, src_id: u8, seq: u8) -> bool {
    cache.is_some_and(|cache| {
        cache.entries.iter().any(|entry| {
            entry.valid
                && entry.packet_type == packet_type
                && entry.src_id == src_id
                && entry.seq == seq
        })
    })
}

pub fn dedupe_accept(cache: Option<&mut Dedupe>, packet_type: u8, src_id: u8, seq: u8) -> bool {
    let Some(cache) = cache else { return false };
    if dedupe_contains(Some(cache), packet_type, src_id, seq) {
        return false;
    }
    cache.entries[usize::from(cache.next)] = PacketKey {
        packet_type,
        src_id,
        seq,
        valid: true,
    };
    cache.next = (cache.next + 1) % DEDUPE_CAPACITY as u8;
    true
}

/// Per-origin audio admission, independent of outer sequence and relay route.
/// Restart requires explicit purge or the existing peer expiry since ADMITTED
/// audio. Rejected replays never renew the age. Legacy byte sequences use the
/// weaker 128 half-window; bundles use u16.
pub fn audio_replay_accept(
    cache: Option<&mut Dedupe>,
    src_id: u8,
    seq: u16,
    legacy: bool,
    now_ms: u32,
) -> bool {
    let Some(cache) = cache.filter(|_| node_id_valid(src_id)) else {
        return false;
    };
    let state = &mut cache.audio[usize::from(src_id - 1)];
    let last = u16::from_le_bytes(state.seq);
    let delta = if legacy {
        u16::from((seq as u8).wrapping_sub(last as u8))
    } else {
        seq.wrapping_sub(last)
    };
    if state.initialized
        && now_ms.wrapping_sub(u32::from_le_bytes(state.admitted_ms))
            <= if legacy {
                2500
            } else {
                crate::adaptive::EXPIRE_MS
            }
        && (delta == 0 || delta >= if legacy { 128 } else { 32768 })
    {
        return false;
    }
    *state = AudioReplay {
        seq: seq.to_le_bytes(),
        admitted_ms: now_ms.to_le_bytes(),
        initialized: true,
    };
    true
}

pub fn dedupe_purge_node(cache: Option<&mut Dedupe>, node_id: u8) {
    if let Some(cache) = cache {
        if node_id_valid(node_id) {
            cache.audio[usize::from(node_id - 1)] = AudioReplay::default();
        }
        for entry in &mut cache.entries {
            if entry.valid && entry.src_id == node_id {
                entry.valid = false;
            }
        }
    }
}

pub fn seq8_reset(state: Option<&mut Seq8>) {
    if let Some(state) = state {
        *state = Seq8::default();
    }
}

pub fn seq16_reset(state: Option<&mut Seq16>) {
    if let Some(state) = state {
        *state = Seq16::default();
    }
}

fn classify(delta: u16, half: u16) -> SeqResult {
    let classification = match delta {
        0 => SeqClass::Duplicate,
        1 => SeqClass::InOrder,
        d if d < half => SeqClass::Gap,
        _ => SeqClass::OldReset,
    };
    SeqResult {
        classification,
        gap: if classification == SeqClass::Gap {
            delta - 1
        } else {
            0
        },
    }
}

pub fn seq8_accept(state: Option<&mut Seq8>, seq: u8) -> SeqResult {
    let first = SeqResult {
        classification: SeqClass::First,
        gap: 0,
    };
    let Some(state) = state else { return first };
    let result = if state.initialized {
        classify(u16::from(seq.wrapping_sub(state.last)), 0x80)
    } else {
        first
    };
    if result.classification != SeqClass::OldReset {
        state.initialized = true;
        state.last = seq;
    }
    result
}

pub fn seq16_accept(state: Option<&mut Seq16>, seq: u16) -> SeqResult {
    let first = SeqResult {
        classification: SeqClass::First,
        gap: 0,
    };
    let Some(state) = state else { return first };
    let result = if state.initialized {
        classify(seq.wrapping_sub(state.last), 0x8000)
    } else {
        first
    };
    if result.classification != SeqClass::OldReset {
        state.initialized = true;
        state.last = seq;
    }
    result
}

/// Compare unsigned bytes, returning exactly -1/0/1. Non-null slices must
/// cover `address_len`; shorter slices panic rather than reading out of bounds.
pub fn address_compare(a: Option<&[u8]>, b: Option<&[u8]>, address_len: usize) -> i8 {
    if address_len == 0 {
        return 0;
    }
    match (a, b) {
        (None, None) => 0,
        (None, Some(_)) => -1,
        (Some(_), None) => 1,
        (Some(a), Some(b)) => match a[..address_len].cmp(&b[..address_len]) {
            ::core::cmp::Ordering::Less => -1,
            ::core::cmp::Ordering::Equal => 0,
            ::core::cmp::Ordering::Greater => 1,
        },
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameBoundaryOverflow;

/// Matches C only where its signed subtraction, multiplication and addition
/// are defined. In particular, `now_us - expected_us` must fit in i64.
pub fn recover_frame_boundary(
    expected_us: i64,
    now_us: i64,
    frame_us: i64,
) -> Result<i64, FrameBoundaryOverflow> {
    if frame_us <= 0 || now_us <= expected_us {
        return Ok(expected_us);
    }
    let elapsed = now_us
        .checked_sub(expected_us)
        .ok_or(FrameBoundaryOverflow)?;
    let advance = (elapsed / frame_us)
        .checked_mul(frame_us)
        .ok_or(FrameBoundaryOverflow)?;
    expected_us
        .checked_add(advance)
        .ok_or(FrameBoundaryOverflow)
}

/// `None` peers represents either C null case: both return zero.
pub fn relay_mask(
    speaker_id: u8,
    local_node_id: u8,
    local_heard_bitmap: u8,
    peers: Option<&[PeerSnapshot]>,
) -> u8 {
    relay_mask_with_peers(
        speaker_id,
        local_node_id,
        local_heard_bitmap,
        peers.into_iter().flatten().copied(),
    )
}

/// Lazy peer fields: inactive/invalid/speaker records need no heard bitmap.
pub trait RelayPeer {
    fn active(&self) -> bool;
    fn node_id(&self) -> u8;
    fn heard_bitmap(&self) -> u8;
}

impl RelayPeer for PeerSnapshot {
    fn active(&self) -> bool {
        self.active
    }
    fn node_id(&self) -> u8 {
        self.node_id
    }
    fn heard_bitmap(&self) -> u8 {
        self.heard_bitmap
    }
}

/// Streaming adapter seam: records are consumed only for a valid speaker.
/// No capacity bound or unique-node normalization is imposed.
pub fn relay_mask_with_peers<P: RelayPeer>(
    speaker_id: u8,
    local_node_id: u8,
    local_heard_bitmap: u8,
    peers: impl IntoIterator<Item = P>,
) -> u8 {
    let speaker_bit = node_bit(speaker_id);
    if speaker_bit == 0 {
        return 0;
    }
    let (mut members, mut heard, mut count) = (0u8, 0u8, 0u8);
    for peer in peers {
        if !peer.active() || !node_id_valid(peer.node_id()) || peer.node_id() == speaker_id {
            continue;
        }
        let bit = node_bit(peer.node_id());
        let bitmap = if peer.node_id() == local_node_id {
            local_heard_bitmap
        } else {
            peer.heard_bitmap()
        };
        members |= bit;
        count = count.wrapping_add(1);
        if bitmap & speaker_bit != 0 {
            heard |= bit;
        }
    }
    if count <= 1 || members & !heard == 0 {
        0
    } else if heard != 0 {
        heard
    } else {
        members
    }
}

/// Output length is the C slot_count; no two-slot cap is imposed. A non-null
/// previous slice must cover the output. Activity index zero is unused.
pub fn select_speakers(
    previous: Option<&[u8]>,
    active_since_ms: Option<&[i64; MAX_NODES + 1]>,
    selected: &mut [u8],
) -> usize {
    struct Slices<'a> {
        previous: Option<&'a [u8]>,
        since: Option<&'a [i64; MAX_NODES + 1]>,
        selected: &'a mut [u8],
    }
    impl SpeakerStorage for Slices<'_> {
        fn inputs_present(&self) -> bool {
            let (Some(previous), Some(_)) = (self.previous, self.since) else {
                return false;
            };
            assert!(previous.len() >= self.selected.len());
            true
        }
        fn previous(&self, index: usize) -> u8 {
            self.previous.unwrap()[index]
        }
        fn activity(&self, node_id: u8) -> i64 {
            self.since.unwrap()[usize::from(node_id)]
        }
        fn selected(&self, index: usize) -> u8 {
            self.selected[index]
        }
        fn set_selected(&mut self, index: usize, value: u8) {
            self.selected[index] = value;
        }
    }
    let slot_count = selected.len();
    select_speakers_with_storage(
        &mut Slices {
            previous,
            since: active_since_ms,
            selected,
        },
        slot_count,
    )
}

/// Storage accesses for speaker selection. Implementations may share backing
/// storage: each read observes all preceding writes. No input is read before
/// the complete output has been zeroed. Indices are bounded by `slot_count`
/// for byte storage and valid node IDs (1..=MAX_NODES) for activity.
pub trait SpeakerStorage {
    /// Checked after zeroing output; false skips all input reads.
    fn inputs_present(&self) -> bool;
    fn previous(&self, index: usize) -> u8;
    fn activity(&self, node_id: u8) -> i64;
    fn selected(&self, index: usize) -> u8;
    fn set_selected(&mut self, index: usize, value: u8);
}

/// Shared allocation-free kernel for slice and alias-preserving ABI storage.
pub fn select_speakers_with_storage(storage: &mut impl SpeakerStorage, slot_count: usize) -> usize {
    fn contains(storage: &impl SpeakerStorage, slot_count: usize, id: u8) -> bool {
        (0..slot_count).any(|i| storage.selected(i) == id)
    }
    for i in 0..slot_count {
        storage.set_selected(i, 0);
    }
    if !storage.inputs_present() {
        return 0;
    }
    let mut count = 0;
    for i in 0..slot_count {
        if count == slot_count {
            break;
        }
        let id = storage.previous(i);
        if node_id_valid(id) && storage.activity(id) > 0 && !contains(storage, slot_count, id) {
            storage.set_selected(count, id);
            count += 1;
        }
    }
    while count < slot_count {
        let (mut best, mut best_since) = (0, 0);
        for id in 1..=MAX_NODES as u8 {
            let since = storage.activity(id);
            if since <= 0 || contains(storage, slot_count, id) {
                continue;
            }
            if best == 0 || since < best_since {
                best = id;
                best_since = since;
            }
        }
        if best == 0 {
            break;
        }
        storage.set_selected(count, best);
        count += 1;
    }
    count
}

pub fn esb_tx_us(
    ramp_us: u32,
    us_per_byte: u32,
    overhead_bytes: u32,
    outer_packet_bytes: u32,
) -> u32 {
    ramp_us.wrapping_add(us_per_byte.wrapping_mul(outer_packet_bytes.wrapping_add(overhead_bytes)))
}

/// First fitting index (C -1 becomes None); input order is not sorted.
pub fn fit_airtime(
    remaining_us: u32,
    margin_us: u32,
    ramp_us: u32,
    us_per_byte: u32,
    overhead_bytes: u32,
    candidate_outer_lens: Option<&[u32]>,
) -> Option<usize> {
    candidate_outer_lens?.iter().position(|&len| {
        esb_tx_us(ramp_us, us_per_byte, overhead_bytes, len).wrapping_add(margin_us) <= remaining_us
    })
}

pub fn defer_local_tail(
    local_pending: bool,
    relay_pending: bool,
    relay_contention_turn: bool,
    tail_is_active_bundle: bool,
) -> bool {
    local_pending && relay_pending && relay_contention_turn && tail_is_active_bundle
}

pub fn successor_carries_prev1(
    deferred_seq: u16,
    tail: Option<&AudioBundleView<'_>>,
    successor: Option<&AudioBundleView<'_>>,
) -> bool {
    struct Views<'a, 'b>(
        Option<&'a AudioBundleView<'b>>,
        Option<&'a AudioBundleView<'b>>,
    );
    impl SuccessorStorage for Views<'_, '_> {
        fn present(&self) -> bool {
            self.0.is_some() && self.1.is_some()
        }
        fn tail_seq(&self) -> u16 {
            self.0.unwrap().current_seq
        }
        fn successor_seq(&self) -> u16 {
            self.1.unwrap().current_seq
        }
        fn tail_flags(&self) -> u8 {
            self.0.unwrap().flags
        }
        fn successor_flags(&self) -> u8 {
            self.1.unwrap().flags
        }
        fn tail_len(&self) -> usize {
            self.0.unwrap().current_data.len()
        }
        fn previous1_len(&self) -> usize {
            self.1.unwrap().previous1_data.len()
        }
        fn payload_equal(&self) -> bool {
            self.1.unwrap().previous1_data == self.0.unwrap().current_data
        }
    }
    successor_carries_prev1_with_storage(deferred_seq, &Views(tail, successor))
}

/// Lazy read-only storage for the successor proof. Methods are called in C
/// short-circuit order; payload_equal is called only after all metadata matches.
/// Implementations may alias views/payloads; no method mutates storage.
pub trait SuccessorStorage {
    fn present(&self) -> bool;
    fn tail_seq(&self) -> u16;
    fn successor_seq(&self) -> u16;
    fn tail_flags(&self) -> u8;
    fn successor_flags(&self) -> u8;
    fn tail_len(&self) -> usize;
    fn previous1_len(&self) -> usize;
    fn payload_equal(&self) -> bool;
}

/// Shared allocation-free proof kernel, including rejection-before-payload order.
pub fn successor_carries_prev1_with_storage(
    deferred_seq: u16,
    storage: &impl SuccessorStorage,
) -> bool {
    storage.present()
        && storage.tail_seq() == deferred_seq
        && storage.successor_seq() == storage.tail_seq().wrapping_add(1)
        && storage.successor_flags() & PREVIOUS1_PRESENT != 0
        && storage.previous1_len() == storage.tail_len()
        && (storage.successor_flags() & PREVIOUS1_ACTIVE != 0)
            == (storage.tail_flags() & CURRENT_ACTIVE != 0)
        && storage.payload_equal()
}

pub fn join_assignment_valid(
    assignment: Option<&JoinAssignment>,
    sender_id: u8,
    expected_coordinator_id: u8,
    slot_count: u8,
) -> bool {
    assignment.is_some_and(|assignment| {
        slot_count > 1
            && usize::from(slot_count) <= MAX_NODES
            && node_id_valid(assignment.assigned_id)
            && assignment.slot_index > 0
            && assignment.slot_index < slot_count
            && node_id_valid(assignment.coordinator_id)
            && assignment.assigned_id != assignment.coordinator_id
            && sender_id == assignment.coordinator_id
            && (expected_coordinator_id == 0
                || expected_coordinator_id == assignment.coordinator_id)
    })
}

/// Writes result only on success, exactly as C. Holes and ignored trailing
/// fields are allowed, but a participant's local slot must be greater than 0.
pub fn slot_map_valid(
    slot_map: Option<&SlotMap>,
    local_node_id: u8,
    coordinator_id: u8,
    result: Option<&mut SlotMapResult>,
) -> bool {
    let Some(map) = slot_map else { return false };
    if !node_id_valid(local_node_id)
        || !node_id_valid(coordinator_id)
        || map.slot_count == 0
        || usize::from(map.slot_count) > MAX_NODES
        || map.slot_ids[0] != coordinator_id
        || usize::from(map.active_speaker_count) > MAX_ACTIVE_SPEAKERS
    {
        return false;
    }
    let mut parsed = SlotMapResult {
        local_slot: -1,
        member_bitmap: 0,
        member_count: 0,
    };
    for (slot, &id) in map.slot_ids[..usize::from(map.slot_count)]
        .iter()
        .enumerate()
    {
        if id == 0 {
            continue;
        }
        let bit = node_bit(id);
        if bit == 0 || parsed.member_bitmap & bit != 0 {
            return false;
        }
        parsed.member_bitmap |= bit;
        parsed.member_count += 1;
        if id == local_node_id {
            parsed.local_slot = slot as i8;
        }
    }
    if parsed.local_slot <= 0 {
        return false;
    }
    let mut speakers = 0;
    for &id in &map.active_speaker_ids[..usize::from(map.active_speaker_count)] {
        let bit = node_bit(id);
        if bit == 0 || parsed.member_bitmap & bit == 0 || speakers & bit != 0 {
            return false;
        }
        speakers |= bit;
    }
    if let Some(result) = result {
        *result = parsed;
    }
    true
}
