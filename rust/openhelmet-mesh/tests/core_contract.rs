use openhelmet_mesh::core::*;
use std::fs;
use std::io;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

mod support;
use support::{TempDir, spawn_group, wait_bounded};

#[cfg(target_os = "linux")]
fn process_is_running(pid: u32) -> bool {
    match fs::read_to_string(format!("/proc/{pid}/status")) {
        Ok(status) => {
            let state = status
                .lines()
                .find_map(|line| line.strip_prefix("State:"))
                .and_then(|state| state.split_whitespace().next())
                .expect("Linux process status must contain a state");
            !matches!(state, "Z" | "X")
        }
        // Linux /proc may report ESRCH when exit races a status-file read.
        Err(error)
            if error.kind() == io::ErrorKind::NotFound || error.raw_os_error() == Some(3) =>
        {
            false
        }
        Err(error) => panic!("could not inspect descendant {pid}: {error}"),
    }
}

#[cfg(target_os = "linux")]
#[test]
fn timeout_kills_sleeping_compiler_descendant() {
    let temp = TempDir::new();
    let pid_path = temp.0.join("descendant.pid");
    let mut command = Command::new("/bin/sh");
    command
        .args([
            "-c",
            "sleep 60 & descendant=$!; printf '%s\\n' \"$descendant\" > \"$1\"; wait \"$descendant\"",
            "mesh-timeout-regression",
        ])
        .arg(&pid_path)
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null());
    let child = spawn_group(&mut command).expect("could not spawn compiler-like shell");
    // Wait for the descendant's PID before starting the short timeout, so
    // slow host scheduling cannot make the regression pass without a child.
    let ready_deadline = Instant::now() + Duration::from_secs(5);
    let descendant = loop {
        if let Ok(pid) = fs::read_to_string(&pid_path) {
            if let Some(pid) = pid
                .strip_suffix('\n')
                .and_then(|pid| pid.parse::<u32>().ok())
            {
                assert!(pid > 0);
                assert_ne!(pid, child.child.id());
                assert!(
                    process_is_running(pid),
                    "descendant must be running before timeout"
                );
                break pid;
            }
        }
        assert!(
            Instant::now() < ready_deadline,
            "descendant did not become ready"
        );
        std::thread::sleep(Duration::from_millis(10));
    };
    let result = wait_bounded(child, Instant::now() + Duration::from_millis(200));
    assert_eq!(result.unwrap_err().kind(), io::ErrorKind::TimedOut);
    let cleanup_deadline = Instant::now() + Duration::from_secs(3);
    while process_is_running(descendant) {
        assert!(
            Instant::now() < cleanup_deadline,
            "descendant survived group timeout cleanup"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
    // An orphan zombie may await init's reaper. Only the shell is directly
    // waitable here; the assertion proves no descendant remains running.
}

fn u16le(dst: &mut Vec<u8>, value: usize) {
    dst.extend_from_slice(&u16::try_from(value).unwrap().to_le_bytes());
}

fn u32le(dst: &mut Vec<u8>, value: u32) {
    dst.extend_from_slice(&value.to_le_bytes());
}

fn i64le(dst: &mut Vec<u8>, value: i64) {
    dst.extend_from_slice(&value.to_le_bytes());
}

#[derive(Default)]
struct Corpus {
    input: Vec<u8>,
    expected: Vec<u8>,
    cases: Vec<(&'static str, usize)>,
    cache: Dedupe,
}

impl Corpus {
    fn finish(&mut self, name: &'static str) {
        self.cases.push((name, self.expected.len()));
    }

    fn ids(&mut self, id: u8, bitmap: u8) {
        let valid = (1..=8).contains(&id);
        let bit = if valid { 1u8 << (id - 1) } else { 0 };
        assert_eq!(node_id_valid(id), valid);
        assert_eq!(node_bit(id), bit);
        assert_eq!(bitmap_contains(bitmap, id), bitmap & bit != 0);
        assert_eq!(slot_for_node_id(id), if valid { id as i8 - 1 } else { -1 });
        let free = (0..8).find(|slot| bitmap & (1 << slot) == 0);
        assert_eq!(first_free_node_id(bitmap), free.map_or(0, |slot| slot + 1));
        self.input.extend_from_slice(&[0, id, bitmap]);
        self.expected.extend_from_slice(&[
            u8::from(node_id_valid(id)),
            node_bit(id),
            u8::from(bitmap_contains(bitmap, id)),
            first_free_node_id(bitmap),
            slot_for_node_id(id) as u8,
        ]);
        self.finish("IDs and bitmaps");
    }

    fn dedupe(&mut self, action: u8, present: bool, kind: u8, src: u8, seq: u8) {
        self.input
            .extend_from_slice(&[1, action, u8::from(present), kind, src, seq]);
        let before = self.cache.snapshot();
        let duplicate = before.entries.iter().any(|entry| {
            entry.valid && (entry.packet_type, entry.src_id, entry.seq) == (kind, src, seq)
        });
        let cache = if present { Some(&mut self.cache) } else { None };
        let value = match action {
            0 => {
                dedupe_reset(cache);
                false
            }
            1 => dedupe_accept(cache, kind, src, seq),
            2 => dedupe_contains(cache.as_deref(), kind, src, seq),
            3 => {
                dedupe_purge_node(cache, src);
                false
            }
            _ => unreachable!(),
        };
        self.expected.push(u8::from(value));
        let snapshot = self.cache.snapshot();
        assert_eq!(
            value,
            present
                && match action {
                    1 => !duplicate,
                    2 => duplicate,
                    _ => false,
                }
        );
        if !present || action == 2 || (action == 1 && duplicate) {
            assert_eq!(
                snapshot, before,
                "absent/read/duplicate operations cannot mutate storage"
            );
        } else if action == 0 {
            assert_eq!(snapshot.next, 0);
            assert_eq!(snapshot.entries, [PacketKey::default(); 32]);
        } else if action == 1 {
            assert_eq!(snapshot.next, (before.next + 1) % 32);
            for (index, entry) in snapshot.entries.iter().enumerate() {
                if index == usize::from(before.next) {
                    assert_eq!(
                        *entry,
                        PacketKey {
                            packet_type: kind,
                            src_id: src,
                            seq,
                            valid: true
                        }
                    );
                } else {
                    assert_eq!(*entry, before.entries[index]);
                }
            }
        } else {
            assert_eq!(
                snapshot.next, before.next,
                "purging cannot advance replacement order"
            );
            for (entry, previous) in snapshot.entries.iter().zip(before.entries) {
                assert_eq!(
                    (entry.packet_type, entry.src_id, entry.seq),
                    (previous.packet_type, previous.src_id, previous.seq)
                );
                assert_eq!(entry.valid, previous.valid && previous.src_id != src);
            }
        }
        self.expected.push(snapshot.next);
        for entry in snapshot.entries {
            self.expected.extend_from_slice(&[
                entry.packet_type,
                entry.src_id,
                entry.seq,
                u8::from(entry.valid),
            ]);
        }
        self.finish("dedupe result and complete post-state");
    }

    fn sequence(&mut self, width: u8, present: bool, reset: bool, init: bool, last: u16, seq: u16) {
        self.input.extend_from_slice(&[
            2,
            width,
            u8::from(present),
            u8::from(reset),
            u8::from(init),
        ]);
        u16le(&mut self.input, usize::from(last));
        u16le(&mut self.input, usize::from(seq));
        let modulus = 1u32 << width;
        let previous = u32::from(last) % modulus;
        let incoming = u32::from(seq) % modulus;
        let distance = (incoming + modulus - previous) % modulus;
        let classification = if !present || reset || !init {
            SeqClass::First
        } else if distance == 0 {
            SeqClass::Duplicate
        } else if distance == 1 {
            SeqClass::InOrder
        } else if distance < modulus / 2 {
            SeqClass::Gap
        } else {
            SeqClass::OldReset
        };
        let (result, last, initialized) = if width == 8 {
            let mut state = Seq8 {
                last: last as u8,
                initialized: init,
            };
            if reset {
                seq8_reset(if present { Some(&mut state) } else { None });
            }
            let result = seq8_accept(if present { Some(&mut state) } else { None }, seq as u8);
            (result, u16::from(state.last), state.initialized)
        } else {
            let mut state = Seq16 {
                last,
                initialized: init,
            };
            if reset {
                seq16_reset(if present { Some(&mut state) } else { None });
            }
            let result = seq16_accept(if present { Some(&mut state) } else { None }, seq);
            (result, state.last, state.initialized)
        };
        assert_eq!(result.classification, classification);
        assert_eq!(
            result.gap,
            if classification == SeqClass::Gap {
                (distance - 1) as u16
            } else {
                0
            }
        );
        assert_eq!(
            u32::from(last),
            if present && classification != SeqClass::OldReset {
                incoming
            } else {
                previous
            }
        );
        assert_eq!(initialized, present || init);
        self.expected.push(result.classification as u8);
        u16le(&mut self.expected, usize::from(result.gap));
        u16le(&mut self.expected, usize::from(last));
        self.expected.push(u8::from(initialized));
        self.finish("sequence classification, gap and post-state");
    }

    fn address(&mut self, a: Option<&[u8]>, b: Option<&[u8]>, len: usize) {
        self.input
            .extend_from_slice(&[3, u8::from(a.is_some()), u8::from(b.is_some())]);
        u16le(&mut self.input, len);
        for slice in [a, b] {
            if let Some(slice) = slice {
                self.input.extend_from_slice(&slice[..len]);
            } else {
                self.input.resize(self.input.len() + len, 0);
            }
        }
        let order = if len == 0 {
            std::cmp::Ordering::Equal
        } else {
            a.map(|bytes| &bytes[..len])
                .cmp(&b.map(|bytes| &bytes[..len]))
        };
        let want = match order {
            std::cmp::Ordering::Less => -1,
            std::cmp::Ordering::Equal => 0,
            std::cmp::Ordering::Greater => 1,
        };
        assert_eq!(address_compare(a, b, len), want);
        self.expected.push(address_compare(a, b, len) as u8);
        self.finish("unsigned address comparison");
    }

    fn boundary(&mut self, expected: i64, now: i64, frame: i64) {
        let result =
            recover_frame_boundary(expected, now, frame).expect("never send C signed overflow");
        if frame <= 0 || now <= expected {
            assert_eq!(result, expected);
        } else {
            assert!(result >= expected && result <= now);
            assert_eq!(
                (i128::from(result) - i128::from(expected)) % i128::from(frame),
                0
            );
            assert!(i128::from(now) - i128::from(result) < i128::from(frame));
        }
        self.input.push(4);
        for value in [expected, now, frame] {
            i64le(&mut self.input, value);
        }
        i64le(&mut self.expected, result);
        self.finish("defined frame boundary arithmetic");
    }

    fn relay(&mut self, speaker: u8, local: u8, heard: u8, peers: Option<&[PeerSnapshot]>) {
        self.input
            .extend_from_slice(&[5, speaker, local, heard, u8::from(peers.is_some())]);
        u16le(&mut self.input, peers.map_or(0, <[PeerSnapshot]>::len));
        if let Some(peers) = peers {
            for peer in peers {
                self.input.extend_from_slice(&[
                    peer.node_id,
                    peer.heard_bitmap,
                    u8::from(peer.active),
                ]);
            }
        }
        let eligible: Vec<_> = peers
            .into_iter()
            .flatten()
            .filter(|peer| {
                peer.active && (1..=8).contains(&peer.node_id) && peer.node_id != speaker
            })
            .collect();
        let member_ids: std::collections::BTreeSet<_> =
            eligible.iter().map(|peer| peer.node_id).collect();
        let hearing_ids: std::collections::BTreeSet<_> = eligible
            .iter()
            .filter(|peer| {
                let observation = if peer.node_id == local {
                    heard
                } else {
                    peer.heard_bitmap
                };
                (1..=8).contains(&speaker) && observation & (1 << (speaker - 1)) != 0
            })
            .map(|peer| peer.node_id)
            .collect();
        // The public API currently counts records in an 8-bit counter, including duplicates.
        let wanted_ids = if !(1..=8).contains(&speaker)
            || eligible.len() % 256 <= 1
            || member_ids == hearing_ids
        {
            std::collections::BTreeSet::new()
        } else if hearing_ids.is_empty() {
            member_ids
        } else {
            hearing_ids
        };
        let wanted = wanted_ids
            .iter()
            .fold(0u8, |bits, id| bits | (1 << (id - 1)));
        assert_eq!(relay_mask(speaker, local, heard, peers), wanted);
        self.expected.push(relay_mask(speaker, local, heard, peers));
        self.finish("relay record counting and bitmap");
    }

    fn speakers(&mut self, previous: Option<&[u8]>, since: Option<&[i64; 9]>, count: usize) {
        self.input
            .extend_from_slice(&[6, u8::from(previous.is_some()), u8::from(since.is_some())]);
        u16le(&mut self.input, count);
        if let Some(previous) = previous {
            self.input.extend_from_slice(&previous[..count]);
        } else {
            self.input.resize(self.input.len() + count, 0);
        }
        for &value in since.unwrap_or(&[0; 9]) {
            i64le(&mut self.input, value);
        }
        let mut selected = vec![0x5a; count];
        let result = select_speakers(previous, since, &mut selected);
        let mut wanted = Vec::new();
        if let (Some(previous), Some(since)) = (previous, since) {
            for &id in &previous[..count] {
                if (1..=8).contains(&id) && since[usize::from(id)] > 0 && !wanted.contains(&id) {
                    wanted.push(id);
                }
            }
            let mut remaining: Vec<u8> = (1..=8)
                .filter(|id| since[usize::from(*id)] > 0 && !wanted.contains(id))
                .collect();
            remaining.sort_by_key(|id| (since[usize::from(*id)], *id));
            wanted.extend(remaining);
            wanted.truncate(count);
        }
        assert_eq!(result, wanted.len());
        wanted.resize(count, 0);
        assert_eq!(
            selected, wanted,
            "retain active incumbents, then oldest activity, then node ID"
        );
        u16le(&mut self.expected, result);
        self.expected.extend_from_slice(&selected);
        self.finish("speaker count and complete zero-filled output");
    }

    fn airtime(&mut self, params: [u32; 5], candidates: Option<&[u32]>) {
        let [remaining, margin, ramp, per, overhead] = params;
        self.input.push(7);
        for value in params {
            u32le(&mut self.input, value);
        }
        self.input.push(u8::from(candidates.is_some()));
        u16le(&mut self.input, candidates.map_or(0, <[u32]>::len));
        for &len in candidates.unwrap_or(&[]) {
            u32le(&mut self.input, len);
        }
        let result = fit_airtime(remaining, margin, ramp, per, overhead, candidates);
        let wire_time = |len: u32| {
            (u128::from(ramp) + u128::from(per) * (u128::from(overhead) + u128::from(len))) as u32
        };
        let wanted = candidates.and_then(|lengths| {
            lengths.iter().position(|len| {
                (u64::from(wire_time(*len)) + u64::from(margin)) as u32 <= remaining
            })
        });
        assert_eq!(result, wanted, "first fitting candidate in supplied order");
        for &len in candidates.unwrap_or(&[]) {
            assert_eq!(esb_tx_us(ramp, per, overhead, len), wire_time(len));
        }
        u32le(
            &mut self.expected,
            result.map_or(u32::MAX, |i| u32::try_from(i).unwrap()),
        );
        for &len in candidates.unwrap_or(&[]) {
            u32le(&mut self.expected, esb_tx_us(ramp, per, overhead, len));
        }
        self.finish("wrapping airtime and first-fit index");
    }

    fn defer(&mut self, bits: u8) {
        assert_eq!(
            defer_local_tail(bits & 1 != 0, bits & 2 != 0, bits & 4 != 0, bits & 8 != 0),
            bits & 15 == 15,
            "all four preconditions are required"
        );
        self.input.extend_from_slice(&[8, bits]);
        self.expected.push(u8::from(defer_local_tail(
            bits & 1 != 0,
            bits & 2 != 0,
            bits & 4 != 0,
            bits & 8 != 0,
        )));
        self.finish("defer truth table");
    }

    fn view(&mut self, view: &AudioBundleView<'_>) {
        u16le(&mut self.input, usize::from(view.current_seq));
        self.input
            .extend_from_slice(&[view.flags, view.codec, view.stream_id]);
        for data in [view.current_data, view.previous1_data, view.previous2_data] {
            u16le(&mut self.input, data.len());
            self.input.extend_from_slice(data);
        }
    }

    fn successor(
        &mut self,
        seq: u16,
        tail: Option<&AudioBundleView<'_>>,
        next: Option<&AudioBundleView<'_>>,
    ) {
        self.input.push(9);
        u16le(&mut self.input, usize::from(seq));
        self.input
            .extend_from_slice(&[u8::from(tail.is_some()), u8::from(next.is_some())]);
        self.view(tail.unwrap_or(&AudioBundleView::default()));
        self.view(next.unwrap_or(&AudioBundleView::default()));
        let recoverable = match (tail, next) {
            (Some(tail), Some(next)) => {
                let adjacent = u32::from(next.current_seq) == (u32::from(seq) + 1) % 65536;
                tail.current_seq == seq
                    && adjacent
                    && next.flags & 2 != 0
                    && (next.flags & 4 != 0) == (tail.flags & 1 != 0)
                    && next.previous1_data == tail.current_data
            }
            _ => false,
        };
        assert_eq!(
            successor_carries_prev1(seq, tail, next),
            recoverable,
            "recovery requires the exact deferred payload, sequence and activity flag"
        );
        self.expected
            .push(u8::from(successor_carries_prev1(seq, tail, next)));
        self.finish("successor proof");
    }

    fn join(&mut self, ack: Option<&JoinAssignment>, sender: u8, expected: u8, slots: u8) {
        let value = ack.copied().unwrap_or_default();
        self.input.extend_from_slice(&[
            10,
            u8::from(ack.is_some()),
            sender,
            expected,
            slots,
            value.assigned_id,
            value.slot_index,
            value.coordinator_id,
        ]);
        let acceptable = ack.is_some()
            && (2..=8).contains(&slots)
            && (1..slots).contains(&value.slot_index)
            && (1..=8).contains(&value.assigned_id)
            && (1..=8).contains(&value.coordinator_id)
            && value.assigned_id != value.coordinator_id
            && sender == value.coordinator_id
            && (expected == 0 || expected == sender);
        assert_eq!(
            join_assignment_valid(ack, sender, expected, slots),
            acceptable
        );
        self.expected.push(u8::from(join_assignment_valid(
            ack, sender, expected, slots,
        )));
        self.finish("join assignment validation");
    }

    fn map(&mut self, map: Option<&SlotMap>, local: u8, coordinator: u8, result_present: bool) {
        self.input.extend_from_slice(&[
            11,
            u8::from(map.is_some()),
            local,
            coordinator,
            u8::from(result_present),
        ]);
        let value = map.copied().unwrap_or_default();
        self.input.push(value.slot_count);
        self.input.extend_from_slice(&value.slot_ids);
        self.input.push(value.active_speaker_count);
        self.input.extend_from_slice(&value.active_speaker_ids);
        self.input.extend_from_slice(&value.relay_masks);
        let mut result = SlotMapResult {
            local_slot: -7,
            member_bitmap: 0xa5,
            member_count: 99,
        };
        let valid = slot_map_valid(
            map,
            local,
            coordinator,
            if result_present {
                Some(&mut result)
            } else {
                None
            },
        );
        let wanted = map.and_then(|map| {
            let slots = map.slot_ids.get(..usize::from(map.slot_count))?;
            let speakers = map
                .active_speaker_ids
                .get(..usize::from(map.active_speaker_count))?;
            if !(1..=8).contains(&local)
                || !(1..=8).contains(&coordinator)
                || slots.first() != Some(&coordinator)
            {
                return None;
            }
            let members: std::collections::BTreeSet<_> =
                slots.iter().copied().filter(|id| *id != 0).collect();
            if members.len() != slots.iter().filter(|id| **id != 0).count()
                || members.iter().any(|id| !(1..=8).contains(id))
            {
                return None;
            }
            let local_slot = slots.iter().position(|id| *id == local)?;
            if local_slot == 0
                || speakers.iter().any(|id| !members.contains(id))
                || speakers
                    .iter()
                    .collect::<std::collections::BTreeSet<_>>()
                    .len()
                    != speakers.len()
            {
                return None;
            }
            Some(SlotMapResult {
                local_slot: local_slot as i8,
                member_bitmap: members.iter().fold(0, |bits, id| bits | (1 << (id - 1))),
                member_count: members.len() as u8,
            })
        });
        assert_eq!(valid, wanted.is_some());
        if result_present && valid {
            assert_eq!(result, wanted.unwrap());
        } else {
            assert_eq!(
                result,
                SlotMapResult {
                    local_slot: -7,
                    member_bitmap: 0xa5,
                    member_count: 99
                }
            );
        }
        self.expected.extend_from_slice(&[
            u8::from(valid),
            result.local_slot as u8,
            result.member_bitmap,
            result.member_count,
        ]);
        self.finish("slot map validity and unchanged-on-error output");
    }
}

fn random(state: &mut u32) -> u32 {
    *state ^= *state << 13;
    *state ^= *state >> 17;
    *state ^= *state << 5;
    *state
}

fn dedupe_corpus(c: &mut Corpus) {
    for present in [false, true] {
        for action in 0..=3 {
            c.dedupe(action, present, 255, 255, 255);
        }
    }
    c.dedupe(0, true, 0, 0, 0);
    for seq in 0..=70 {
        c.dedupe(1, true, 255, if seq % 3 == 0 { 0 } else { 255 }, seq);
        c.dedupe(1, true, 255, if seq % 3 == 0 { 0 } else { 255 }, seq);
        c.dedupe(2, true, 255, 0, 0);
        if seq == 31 || seq == 34 {
            c.dedupe(3, true, 0, 255, 0);
        }
    }
    c.dedupe(0, true, 0, 0, 0);
    let mut seed = 0x8a24_4917;
    for i in 0..3000 {
        let r = random(&mut seed);
        let action = if i % 301 == 0 { 0 } else { 1 + (r % 3) as u8 };
        c.dedupe(
            action,
            r & 0x80 != 0,
            (r >> 8) as u8,
            (r >> 16) as u8,
            (r >> 24) as u8,
        );
    }
}

fn sequence_corpus(c: &mut Corpus) {
    for last in 0..=255 {
        for seq in 0..=255 {
            c.sequence(8, true, false, true, last, seq);
        }
    }
    for last in [0u16, 1, 127, 128, 255, 256, 32767, 32768, 65534, 65535] {
        for delta in [
            0u16, 1, 2, 127, 128, 32766, 32767, 32768, 32769, 65534, 65535,
        ] {
            c.sequence(16, true, false, true, last, last.wrapping_add(delta));
        }
    }
    for width in [8, 16] {
        for present in [false, true] {
            for reset in [false, true] {
                for init in [false, true] {
                    c.sequence(width, present, reset, init, 250, 3);
                }
            }
        }
    }
    // Chained OLD_RESET must set last, making the following packet in-order.
    let mut s8 = Seq8 {
        last: 20,
        initialized: true,
    };
    for seq in [19, 20, 255, 0, 127, 255, 0] {
        c.sequence(
            8,
            true,
            false,
            s8.initialized,
            u16::from(s8.last),
            u16::from(seq),
        );
        seq8_accept(Some(&mut s8), seq);
    }
    let mut s16 = Seq16 {
        last: 20,
        initialized: true,
    };
    for seq in [19, 20, 65535, 0, 32767, 65535, 0] {
        c.sequence(16, true, false, s16.initialized, s16.last, seq);
        seq16_accept(Some(&mut s16), seq);
    }
}

fn relay_corpus(c: &mut Corpus) {
    let mut peers = [PeerSnapshot {
        node_id: 1,
        heard_bitmap: 0,
        active: true,
    }; 4];
    for (i, peer) in peers.iter_mut().enumerate() {
        peer.node_id = i as u8 + 1;
    }
    peers[2].heard_bitmap = 2;
    for count in 0..=4 {
        for heard in [0, 2, 255] {
            c.relay(2, 1, heard, Some(&peers[..count]));
        }
    }
    c.relay(0, 1, 0, Some(&peers));
    c.relay(255, 1, 0, Some(&peers));
    c.relay(2, 1, 0, None);
    for count in [1, 2, 255, 256, 257, 258, 511, 512, 513] {
        let records = vec![
            PeerSnapshot {
                node_id: 3,
                heard_bitmap: 0,
                active: true
            };
            count
        ];
        c.relay(2, 1, 0, Some(&records));
    }
    // Duplicate records are not collapsed; one heard record covers its node bit.
    c.relay(
        2,
        1,
        0,
        Some(&[
            PeerSnapshot {
                node_id: 3,
                heard_bitmap: 2,
                active: true,
            },
            PeerSnapshot {
                node_id: 3,
                heard_bitmap: 0,
                active: true,
            },
            PeerSnapshot {
                node_id: 4,
                heard_bitmap: 0,
                active: true,
            },
            PeerSnapshot {
                node_id: 0,
                heard_bitmap: 255,
                active: true,
            },
            PeerSnapshot {
                node_id: 255,
                heard_bitmap: 255,
                active: true,
            },
            PeerSnapshot {
                node_id: 5,
                heard_bitmap: 2,
                active: false,
            },
        ]),
    );
    for bitmap in 0..=255 {
        for speaker in 1..=8 {
            c.relay(speaker, 3, bitmap, Some(&peers));
        }
    }
}

fn speaker_corpus(c: &mut Corpus) {
    let since = [i64::MAX, 50, 0, 400, -1, 300, 400, 100, 1];
    let previous = [9, 6, 6, 2, 7, 255, 8, 1, 0, 5, 3, 0];
    for count in 0..=previous.len() {
        c.speakers(Some(&previous), Some(&since), count);
        c.speakers(None, Some(&since), count);
        c.speakers(Some(&previous), None, count);
        c.speakers(Some(&previous), Some(&[0; 9]), count);
        c.speakers(Some(&[0; 12]), Some(&[999, 1, 1, 1, 1, 1, 1, 1, 1]), count);
    }
    let mut seed = 0x29ab_cdef;
    for _ in 0..500 {
        let mut since = [0i64; 9];
        let mut previous = [0u8; 10];
        for time in &mut since {
            *time = i64::from(random(&mut seed) % 7) - 2;
        }
        for id in &mut previous {
            *id = (random(&mut seed) % 12) as u8;
        }
        let count = (random(&mut seed) % 11) as usize;
        c.speakers(Some(&previous), Some(&since), count);
    }
}

fn successor_corpus(c: &mut Corpus) {
    let frame = [0x11, 0x22, 0x80, 0xff];
    let other = [0x11, 0x22, 0x80, 0xfe];
    for seq in [0u16, 41, 65535] {
        for active in [false, true] {
            let tail = AudioBundleView {
                current_data: &frame,
                current_seq: seq,
                flags: if active { CURRENT_ACTIVE } else { 0 },
                codec: 1,
                stream_id: 3,
                ..AudioBundleView::default()
            };
            let next = AudioBundleView {
                current_data: &other,
                current_seq: seq.wrapping_add(1),
                previous1_data: &frame,
                flags: PREVIOUS1_PRESENT | if active { PREVIOUS1_ACTIVE } else { 0 },
                codec: 255,
                stream_id: 200,
                previous2_data: &other,
            };
            c.successor(seq, Some(&tail), Some(&next));
            c.successor(seq.wrapping_sub(1), Some(&tail), Some(&next));
            c.successor(seq, None, Some(&next));
            c.successor(seq, Some(&tail), None);
            c.successor(seq, None, None);
            let mut bad = next;
            bad.current_seq = seq.wrapping_add(2);
            c.successor(seq, Some(&tail), Some(&bad));
            bad = next;
            bad.flags &= !PREVIOUS1_PRESENT;
            c.successor(seq, Some(&tail), Some(&bad));
            bad = next;
            bad.flags ^= PREVIOUS1_ACTIVE;
            c.successor(seq, Some(&tail), Some(&bad));
            bad = next;
            bad.previous1_data = &frame[..3];
            c.successor(seq, Some(&tail), Some(&bad));
            bad = next;
            bad.previous1_data = &other;
            c.successor(seq, Some(&tail), Some(&bad));
            // Other flags, current payload, codec, stream and prev2 are ignored.
            for flags in 0..=255 {
                bad = next;
                bad.flags = flags;
                c.successor(seq, Some(&tail), Some(&bad));
            }
        }
    }
    let tail = AudioBundleView {
        current_seq: 1,
        ..AudioBundleView::default()
    };
    let next = AudioBundleView {
        current_seq: 2,
        flags: PREVIOUS1_PRESENT,
        ..AudioBundleView::default()
    };
    c.successor(1, Some(&tail), Some(&next));
}

fn map_join_corpus(c: &mut Corpus) {
    let ack = JoinAssignment {
        assigned_id: 3,
        slot_index: 5,
        coordinator_id: 1,
    };
    c.join(None, 1, 1, 8);
    for assigned in [0, 1, 2, 3, 8, 9, 255] {
        for slot in [0, 1, 2, 5, 7, 8, 255] {
            for coord in [0, 1, 3, 8, 9, 255] {
                for count in [0, 1, 2, 8, 9, 255] {
                    let value = JoinAssignment {
                        assigned_id: assigned,
                        slot_index: slot,
                        coordinator_id: coord,
                    };
                    c.join(Some(&value), coord, 0, count);
                }
            }
        }
    }
    for sender in 0..=255 {
        c.join(Some(&ack), sender, 1, 8);
        c.join(Some(&ack), 1, sender, 8);
    }
    let base = SlotMap {
        slot_count: 5,
        slot_ids: [1, 0, 3, 0, 5, 255, 255, 255],
        active_speaker_count: 2,
        active_speaker_ids: [3, 5],
        relay_masks: [255, 255],
    };
    c.map(None, 3, 1, true);
    c.map(Some(&base), 3, 1, false);
    for local in 0..=255 {
        c.map(Some(&base), local, 1, true);
    }
    for coord in 0..=255 {
        c.map(Some(&base), 3, coord, true);
    }
    for count in 0..=255 {
        let mut map = base;
        map.slot_count = count;
        c.map(Some(&map), 3, 1, true);
        map = base;
        map.active_speaker_count = count;
        c.map(Some(&map), 3, 1, true);
    }
    for index in 0..8 {
        for id in 0..=255 {
            let mut map = base;
            map.slot_ids[index] = id;
            c.map(Some(&map), 3, 1, true);
        }
    }
    for index in 0..2 {
        for id in 0..=255 {
            let mut map = base;
            map.active_speaker_ids[index] = id;
            c.map(Some(&map), 3, 1, true);
            map.active_speaker_count = index as u8;
            c.map(Some(&map), 3, 1, true);
            map = base;
            map.relay_masks[index] = id;
            c.map(Some(&map), 3, 1, true);
        }
    }
    let mut seed = 0x2345_6789;
    for _ in 0..1000 {
        let mut map = SlotMap {
            slot_count: (random(&mut seed) % 10) as u8,
            active_speaker_count: (random(&mut seed) % 4) as u8,
            ..SlotMap::default()
        };
        for id in &mut map.slot_ids {
            *id = (random(&mut seed) % 11) as u8;
        }
        for id in &mut map.active_speaker_ids {
            *id = (random(&mut seed) % 11) as u8;
        }
        c.map(
            Some(&map),
            (random(&mut seed) % 11) as u8,
            (random(&mut seed) % 11) as u8,
            true,
        );
    }
}

fn corpus() -> Corpus {
    let mut c = Corpus::default();
    for id in 0..=255 {
        for bitmap in 0..=255 {
            c.ids(id, bitmap);
        }
    }
    dedupe_corpus(&mut c);
    sequence_corpus(&mut c);
    for a in 0..=255 {
        for b in 0..=255 {
            c.address(Some(&[a]), Some(&[b]), 1);
        }
    }
    for len in [0, 1, 5, 6, 32] {
        let a = vec![0x80; len];
        let mut b = a.clone();
        c.address(Some(&a), Some(&a), len);
        for index in 0..len {
            b[index] = 0xff;
            c.address(Some(&a), Some(&b), len);
            b[index] = 0x80;
        }
        c.address(None, Some(&a), len);
        c.address(Some(&a), None, len);
        c.address(None, None, len);
    }
    for expected in [i64::MIN, -1000000, -1, 0, 1000000, i64::MAX - 100000] {
        for frame in [-1, 0, 1, 2, 20000, i64::MAX] {
            for delta in [0i64, 1, 10000, 20000, 25000, 65000] {
                c.boundary(expected, expected.checked_add(delta).unwrap(), frame);
            }
        }
    }
    c.boundary(10, -10, 20);
    c.boundary(i64::MIN, i64::MAX, 0);
    c.boundary(i64::MIN, i64::MIN + i64::MAX, 1);
    relay_corpus(&mut c);
    speaker_corpus(&mut c);
    for remaining in [0, 632, 633, 872, 873, 1104, 1105, 5000, u32::MAX] {
        c.airtime([remaining, 100, 129, 4, 11], Some(&[208, 150, 90]));
        c.airtime([remaining, 100, 129, 4, 11], Some(&[90, 208, 150]));
        c.airtime([remaining, 100, 129, 4, 11], Some(&[208]));
        c.airtime([remaining, 100, 129, 4, 11], Some(&[]));
        c.airtime([remaining, 100, 129, 4, 11], None);
    }
    for ramp in [0, 1, u32::MAX] {
        for per in [0, 1, 4, u32::MAX] {
            for overhead in [0, 11, u32::MAX] {
                for margin in [0, 100, u32::MAX] {
                    c.airtime(
                        [100, margin, ramp, per, overhead],
                        Some(&[u32::MAX, 1, 0, 208]),
                    );
                }
            }
        }
    }
    for bits in 0..16 {
        c.defer(bits);
    }
    successor_corpus(&mut c);
    map_join_corpus(&mut c);
    c
}

#[test]
fn corpus_checks_core_requirements_and_complete_poststate() {
    let corpus = corpus();
    assert!(!corpus.expected.is_empty());
    eprintln!("mesh core corpus: {} cases", corpus.cases.len());
}

#[test]
fn explicit_contracts_and_c_undefined_overflow_exclusion() {
    assert_eq!(MAX_NODES, 8);
    assert_eq!(MAX_ACTIVE_SPEAKERS, 2);
    assert_eq!(DEDUPE_CAPACITY, 32);
    assert_eq!(
        recover_frame_boundary(i64::MIN, i64::MAX, 1),
        Err(FrameBoundaryOverflow)
    );
    assert_eq!(
        recover_frame_boundary(-1, i64::MAX, i64::MAX),
        Err(FrameBoundaryOverflow)
    );
    assert_eq!(recover_frame_boundary(i64::MIN, i64::MAX, 0), Ok(i64::MIN));
    assert_eq!(recover_frame_boundary(i64::MAX, i64::MIN, 1), Ok(i64::MAX));
    assert_eq!(recover_frame_boundary(-10, 15, 20), Ok(10));
    let mut s8 = Seq8 {
        last: 0,
        initialized: true,
    };
    assert_eq!(
        seq8_accept(Some(&mut s8), 128).classification,
        SeqClass::OldReset
    );
    assert_eq!(s8.last, 0);
    assert_eq!(
        seq8_accept(Some(&mut s8), 129).classification,
        SeqClass::OldReset
    );
    let mut s16 = Seq16 {
        last: 0,
        initialized: true,
    };
    assert_eq!(
        seq16_accept(Some(&mut s16), 32768).classification,
        SeqClass::OldReset
    );
    assert_eq!(s16.last, 0);
    assert_eq!(
        seq16_accept(Some(&mut s16), 32769).classification,
        SeqClass::OldReset
    );
    let mut cache = Dedupe::default();
    assert!(dedupe_accept(Some(&mut cache), 255, 0, 255));
    let before = cache.snapshot();
    dedupe_purge_node(Some(&mut cache), 0);
    let after = cache.snapshot();
    assert_eq!(before.next, after.next);
    assert_eq!(before.entries[0].packet_type, after.entries[0].packet_type);
    assert_eq!(before.entries[0].seq, after.entries[0].seq);
    assert!(!after.entries[0].valid);
    let mut output = [255; 10];
    assert_eq!(
        select_speakers(
            Some(&[0; 10]),
            Some(&[999, 1, 1, 1, 1, 1, 1, 1, 1]),
            &mut output
        ),
        8
    );
    assert_eq!(output, [1, 2, 3, 4, 5, 6, 7, 8, 0, 0]);
    assert_eq!(fit_airtime(10, 0, 0, 1, 0, Some(&[9, 1])), Some(0));
    assert_eq!(esb_tx_us(u32::MAX, 1, u32::MAX, 1), u32::MAX);
    let ack = JoinAssignment {
        assigned_id: 3,
        slot_index: 5,
        coordinator_id: 1,
    };
    assert!(join_assignment_valid(Some(&ack), 1, 0, 8));
}
