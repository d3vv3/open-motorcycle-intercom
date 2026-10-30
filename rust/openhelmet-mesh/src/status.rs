//! Stateless STATUS evidence and wire projections. Storage and locking remain C.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Projection {
    pub battery: u8,
    pub peers: u8,
    pub firmware: u8,
    pub temperature: i8,
    pub heard: u8,
    pub relayed: u8,
}
const fn bit(id: u8) -> u16 {
    if id >= 1 && id <= 8 { 1 << (id - 1) } else { 0 }
}
pub const fn restore(evidence: u16, credit: u16) -> u16 {
    evidence | credit
}
pub const fn scrub(evidence: u16, id: u8) -> u16 {
    evidence & !(bit(id) | (bit(id) << 8))
}
pub const fn heard(evidence: u16, id: u8, active: bool) -> u16 {
    evidence | if active { bit(id) } else { 0 }
}
pub const fn relay(evidence: u16, id: u8) -> u16 {
    evidence | (bit(id) << 8)
}
pub const fn clear() -> u16 {
    0
}
pub const fn local_credit(
    kind: u8,
    len: u16,
    source: u8,
    local: u8,
    flags: u8,
    heard: u8,
    relayed: u8,
) -> u16 {
    if kind == 7 && len == 17 && source == local && flags & 2 == 0 {
        heard as u16 | ((relayed as u16) << 8)
    } else {
        0
    }
}
pub const fn keepalive() -> [u8; 2] {
    [100, 0]
}
pub const fn build(count: u8, version: u8, heard: u8, relayed: u8, speakers: u8) -> [u8; 8] {
    [100, 127, count, version, 127, heard, relayed, speakers]
}
pub fn decode_status(bytes: &[u8]) -> Option<Projection> {
    let b: &[u8; 8] = bytes.try_into().ok()?;
    Some(Projection {
        battery: b[0],
        peers: b[2],
        firmware: b[3],
        temperature: b[4] as i8,
        heard: b[5],
        relayed: b[6],
    })
}
pub fn decode_keepalive(bytes: &[u8]) -> Option<u8> {
    let b: &[u8; 2] = bytes.try_into().ok()?;
    Some(b[0])
}
