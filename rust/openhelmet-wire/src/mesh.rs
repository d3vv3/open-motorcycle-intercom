//! Byte layouts only; transport dispatch and control policy are not validators here.

pub const VERSION: u8 = 5;
pub const HEADER_SIZE: usize = 9;
pub const CONTROL_MAX_PACKET_SIZE: usize = 209;
pub const NRF_RECEIVE_PAYLOAD_CAPACITY: usize = 200;
pub const ESP_RECEIVE_PAYLOAD_CAPACITY: usize = 128;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Length,
    Version,
    Capacity,
    UnknownType,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    pub version: u8,
    pub packet_type: u8,
    pub src_id: u8,
    pub seq: u8,
    pub ttl: u8,
    pub flags: u8,
    pub talk_channel: u8,
    pub payload_len: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Envelope<'a> {
    pub header: Header,
    pub payload: &'a [u8],
}

/// Decode exactly one version-5 envelope. Unknown types and reserved bits survive.
/// Capacity is the caller's payload storage limit, excluding the nine-byte
/// header. It is independent of the total control-packet limit and dispatch policy.
pub fn decode(data: &[u8], payload_capacity: usize) -> Result<Envelope<'_>, Error> {
    if data.len() < HEADER_SIZE {
        return Err(Error::Length);
    }
    if data[0] != VERSION {
        return Err(Error::Version);
    }
    let payload_len = u16::from_le_bytes([data[7], data[8]]);
    if data.len() != HEADER_SIZE + usize::from(payload_len) {
        return Err(Error::Length);
    }
    if usize::from(payload_len) > payload_capacity {
        return Err(Error::Capacity);
    }
    Ok(Envelope {
        header: Header {
            version: data[0],
            packet_type: data[1],
            src_id: data[2],
            seq: data[3],
            ttl: data[4],
            flags: data[5],
            talk_channel: data[6],
            payload_len,
        },
        payload: &data[HEADER_SIZE..],
    })
}

/// Header length must agree with payload. Capacity excludes the header;
/// output storage bounds the total packet. Errors never mutate output.
pub fn encode(
    header: &Header,
    payload: &[u8],
    output: &mut [u8],
    payload_capacity: usize,
) -> Result<usize, Error> {
    if header.version != VERSION {
        return Err(Error::Version);
    }
    if payload.len() != usize::from(header.payload_len) {
        return Err(Error::Length);
    }
    let len = HEADER_SIZE + payload.len();
    if payload.len() > payload_capacity || len > output.len() {
        return Err(Error::Capacity);
    }
    let length = header.payload_len.to_le_bytes();
    output[..HEADER_SIZE].copy_from_slice(&[
        header.version,
        header.packet_type,
        header.src_id,
        header.seq,
        header.ttl,
        header.flags,
        header.talk_channel,
        length[0],
        length[1],
    ]);
    output[HEADER_SIZE..len].copy_from_slice(payload);
    Ok(len)
}

// Field codecs avoid packed references, padding, native endian, and allocation.
trait Field: Sized {
    const SIZE: usize;
    fn read(data: &mut &[u8]) -> Self;
    fn write(&self, data: &mut &mut [u8]);
}
macro_rules! primitive {
    ($t:ty, $n:expr) => {
        impl Field for $t {
            const SIZE: usize = $n;
            fn read(data: &mut &[u8]) -> Self {
                let (head, tail) = data.split_at(Self::SIZE);
                *data = tail;
                Self::from_le_bytes(head.try_into().expect("checked field size"))
            }
            fn write(&self, data: &mut &mut [u8]) {
                let (head, tail) = core::mem::take(data).split_at_mut(Self::SIZE);
                head.copy_from_slice(&self.to_le_bytes());
                *data = tail;
            }
        }
    };
}
primitive!(u8, 1);
primitive!(i8, 1);
primitive!(u16, 2);
primitive!(i16, 2);
primitive!(u32, 4);
impl<T: Field, const N: usize> Field for [T; N] {
    const SIZE: usize = T::SIZE * N;
    fn read(data: &mut &[u8]) -> Self {
        core::array::from_fn(|_| T::read(data))
    }
    fn write(&self, data: &mut &mut [u8]) {
        for field in self {
            field.write(data);
        }
    }
}

macro_rules! layout {
    ($name:ident { $($field:ident: $ty:ty),* $(,)? }) => {
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        pub struct $name { $(pub $field: $ty),* }
        impl Field for $name {
            const SIZE: usize = 0 $(+ <$ty as Field>::SIZE)*;
            fn read(data: &mut &[u8]) -> Self { Self { $($field: <$ty as Field>::read(data)),* } }
            fn write(&self, data: &mut &mut [u8]) { $(self.$field.write(data);)* }
        }
        impl $name {
            pub const SIZE: usize = <Self as Field>::SIZE;
            pub fn decode(data: &[u8]) -> Result<Self, Error> {
                if data.len() != Self::SIZE { return Err(Error::Length); }
                Ok(<Self as Field>::read(&mut &data[..]))
            }
            pub fn encode(&self, output: &mut [u8]) -> Result<usize, Error> {
                if output.len() < Self::SIZE { return Err(Error::Capacity); }
                self.write(&mut &mut output[..Self::SIZE]);
                Ok(Self::SIZE)
            }
        }
    };
}
layout!(Identity {
    address_len: u8,
    address: [u8; 6]
});
layout!(Join {
    capabilities: u8,
    reserved: u8
});
layout!(JoinAck {
    assigned_id: u8,
    slot_index: u8,
    coordinator_id: u8
});
layout!(Keepalive {
    battery_pct: u8,
    reserved: u8
});
layout!(SlotMap {
    slot_count: u8,
    slot_ids: [u8; 8],
    active_speaker_count: u8,
    active_speaker_ids: [u8; 2],
    relay_masks: [u8; 2]
});
layout!(MembershipV3 {
    term: u32,
    revision: u32,
    leader_id: u8,
    slot_map: SlotMap,
    identities: [Identity; 8]
});
layout!(JoinV3 {
    origin: Identity,
    capabilities: u8,
    target: Identity
});
layout!(JoinAckV3 {
    target: Identity,
    capabilities: u8,
    assigned_id: u8,
    slot_index: u8,
    coordinator_id: u8,
    term: u32
});
layout!(Topology {
    report_seq: u32,
    origin: Identity,
    direct_mask: u8,
    rssi_dbm: [i8; 8],
    quality: [u8; 8],
    leader_id: u8,
    term: u32,
    slot_map: SlotMap
});
layout!(SpeakerRequest {
    term: u32,
    request_seq: u32,
    frame_counter: u32,
    active: u8
});
layout!(SyncV3 {
    leader: Identity,
    leader_id: u8,
    member_count: u8,
    term: u32,
    frame_counter: u32,
    phase_us: u16,
    relay_depth: u8
});
layout!(HandoverProposal {
    old_leader_id: u8,
    candidate_id: u8,
    next_term: u32,
    switch_frame: u32,
    members: u8,
    slot_map: SlotMap
});
layout!(HandoverAck {
    old_leader_id: u8,
    candidate_id: u8,
    next_term: u32,
    switch_frame: u32,
    acknowledger_id: u8
});
layout!(Status {
    battery_pct: u8,
    rssi_dbm: i8,
    peer_count: u8,
    fw_version: u8,
    temperature_c: i8,
    heard_bitmap: u8,
    relay_bitmap: u8,
    active_speakers: u8
});
layout!(SpeakerGrant {
    speaker_count: u8,
    speaker_ids: [u8; 2],
    relay_masks: [u8; 2]
});
layout!(SpeakerRelease {
    speaker_count: u8,
    speaker_ids: [u8; 2]
});
layout!(JoinV2 {
    capabilities: u8,
    reserved: u8,
    requester_addr: [u8; 5]
});
layout!(JoinAckV2 {
    assigned_id: u8,
    slot_index: u8,
    coordinator_id: u8,
    target_addr: [u8; 5]
});
layout!(SyncNrf {
    frame_counter: u32,
    drift_ppm: i16,
    coordinator_addr: [u8; 5]
});
layout!(SyncEsp {
    frame_counter: u32,
    drift_ppm: i16,
    coordinator_mac: [u8; 6]
});
layout!(LeaveNrf {
    sender_addr: [u8; 5]
});

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Audio<'a> {
    pub codec: u8,
    pub frame_ms: u8,
    pub stream_id: u8,
    pub audio_flags: u8,
    pub data: &'a [u8],
}
impl<'a> Audio<'a> {
    /// Layout interpretation accepts a four-byte prefix and variable data through
    /// the 200-byte bundle capacity. Codec-specific validation is separate.
    pub fn decode(data: &'a [u8]) -> Result<Self, Error> {
        if !(4..=200).contains(&data.len()) {
            return Err(Error::Length);
        }
        Ok(Self {
            codec: data[0],
            frame_ms: data[1],
            stream_id: data[2],
            audio_flags: data[3],
            data: &data[4..],
        })
    }
    pub fn encode(&self, output: &mut [u8]) -> Result<usize, Error> {
        if self.data.len() > 196 {
            return Err(Error::Length);
        }
        let len = 4 + self.data.len();
        if output.len() < len {
            return Err(Error::Capacity);
        }
        output[..4].copy_from_slice(&[self.codec, self.frame_ms, self.stream_id, self.audio_flags]);
        output[4..len].copy_from_slice(self.data);
        Ok(len)
    }
}

/// Exact counterpart of the shared C helper; data_len excludes the legacy
/// prefix and any end-to-end sequence bytes handled by the calling firmware.
pub fn audio_wire_validate(codec: u8, frame_ms: u8, data_len: u16, local_codec: u8) -> bool {
    frame_ms == 20
        && codec == local_codec
        && match codec {
            2 => data_len == 48,
            1 => data_len > 0 && data_len <= 64,
            _ => false,
        }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Transport {
    Nrf,
    Esp,
}

macro_rules! messages {
    ($( $variant:ident($ty:ty) = $id:literal ),* $(,)?) => {
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        pub enum Message<'a> {
            $($variant($ty),)*
            Audio(Audio<'a>), AudioV2(crate::audio_bundle::BundleView<'a>),
            Leave, LeaveNrf(LeaveNrf), SyncNrf(SyncNrf), SyncEsp(SyncEsp),
        }
        impl<'a> Message<'a> {
            pub fn decode(packet_type: u8, data: &'a [u8], transport: Transport) -> Result<Self, Error> {
                match packet_type {
                    $($id => Ok(Self::$variant(<$ty>::decode(data)?)),)*
                    1 => Ok(Self::Audio(Audio::decode(data)?)),
                    4 if data.is_empty() => Ok(Self::Leave),
                    4 if transport == Transport::Nrf => Ok(Self::LeaveNrf(LeaveNrf::decode(data)?)),
                    4 => Err(Error::Length),
                    5 => match transport {
                        Transport::Nrf => Ok(Self::SyncNrf(SyncNrf::decode(data)?)),
                        Transport::Esp => Ok(Self::SyncEsp(SyncEsp::decode(data)?)),
                    },
                    13 => crate::audio_bundle::audio_bundle_parse(data).map(Self::AudioV2).ok_or(Error::Length),
                    _ => Err(Error::UnknownType),
                }
            }
            pub fn packet_type(&self) -> u8 {
                match self {
                    $(Self::$variant(_) => $id,)*
                    Self::Audio(_) => 1, Self::Leave | Self::LeaveNrf(_) => 4,
                    Self::SyncNrf(_) | Self::SyncEsp(_) => 5, Self::AudioV2(_) => 13,
                }
            }
            pub fn encode(&self, output: &mut [u8]) -> Result<usize, Error> {
                match self {
                    $(Self::$variant(value) => value.encode(output),)*
                    Self::Audio(value) => value.encode(output), Self::Leave => Ok(0),
                    Self::LeaveNrf(value) => value.encode(output),
                    Self::SyncNrf(value) => value.encode(output), Self::SyncEsp(value) => value.encode(output),
                    Self::AudioV2(value) => {
                        let len = crate::audio_bundle::audio_bundle_encode(value, output);
                        if len == 0 { Err(Error::Length) } else { Ok(len) }
                    }
                }
            }
        }
    };
}
messages! {
    Join(Join) = 2, JoinAck(JoinAck) = 3, SlotMap(SlotMap) = 6,
    Status(Status) = 7, Keepalive(Keepalive) = 8, SpeakerGrant(SpeakerGrant) = 9,
    SpeakerRelease(SpeakerRelease) = 10, JoinV2(JoinV2) = 11, JoinAckV2(JoinAckV2) = 12,
    Topology(Topology) = 14, JoinV3(JoinV3) = 15, JoinAckV3(JoinAckV3) = 16,
    SyncV3(SyncV3) = 17, HandoverPrepare(HandoverProposal) = 18,
    HandoverAck(HandoverAck) = 19, HandoverCommit(HandoverProposal) = 20,
    HandoverCancel(HandoverProposal) = 21, MembershipV3(MembershipV3) = 22,
    SpeakerRequest(SpeakerRequest) = 23,
}
