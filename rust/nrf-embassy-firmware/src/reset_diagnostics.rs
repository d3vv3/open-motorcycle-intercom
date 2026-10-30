//! Portable retained-record validation; all raw RAM access belongs to the binary.

const MAGIC: u32 = 0x5253_5431;
pub const COUNTER_MAGIC: u32 = 0x424f_4f54;
pub const RECORD_WORDS: usize = 9;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Breadcrumb {
    pub kind: u32,
    pub file_hash: u32,
    pub line: u32,
    pub column: u32,
    pub pc: u32,
    pub lr: u32,
    pub boot: u32,
}

impl Breadcrumb {
    pub fn encode(self) -> [u32; RECORD_WORDS] {
        let mut words = [
            MAGIC,
            self.kind,
            self.file_hash,
            self.line,
            self.column,
            self.pc,
            self.lr,
            self.boot,
            0,
        ];
        words[8] = checksum(&words[..8]);
        words
    }

    pub fn decode(words: [u32; RECORD_WORDS]) -> Option<Self> {
        if words[0] != MAGIC || !matches!(words[1], 1 | 2) || words[8] != checksum(&words[..8]) {
            return None;
        }
        Some(Self {
            kind: words[1],
            file_hash: words[2],
            line: words[3],
            column: words[4],
            pc: words[5],
            lr: words[6],
            boot: words[7],
        })
    }
}

fn checksum(words: &[u32]) -> u32 {
    words
        .iter()
        .fold(0x9e37_79b9, |sum, word| sum.rotate_left(5) ^ word)
}

pub fn file_hash(file: &str) -> u32 {
    file.bytes().fold(0x811c_9dc5u32, |hash, byte| {
        (hash ^ u32::from(byte)).wrapping_mul(0x0100_0193)
    })
}

pub fn next_boot(magic: u32, boot: u32) -> u32 {
    if magic == COUNTER_MAGIC {
        boot.wrapping_add(1)
    } else {
        1
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Snapshot {
    pub resetreas: u32,
    pub boots: u32,
    pub breadcrumb: Option<Breadcrumb>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reset_rows_are_ascii_bounded_and_keep_field_order_at_max_width() {
        use crate::diagnostics::DiagnosticLine;
        for kind in [0, 1, 2] {
            let s = Snapshot {
                resetreas: u32::MAX,
                boots: u32::MAX,
                breadcrumb: (kind != 0).then_some(Breadcrumb {
                    kind,
                    file_hash: u32::MAX,
                    line: u32::MAX,
                    column: u32::MAX,
                    pc: u32::MAX,
                    lr: u32::MAX,
                    boot: u32::MAX,
                }),
            };
            let line = DiagnosticLine::reset(s).unwrap();
            let bytes = line.as_bytes();
            assert!(bytes.is_ascii());
            assert!(bytes.len() <= 896);
            assert_eq!(bytes.last(), Some(&b'\n'));
            let text = core::str::from_utf8(bytes).unwrap();
            let mut fields = text.split_ascii_whitespace();
            assert_eq!(fields.next(), Some("PIPE_RESET"));
            for key in [
                "v",
                "resetreas",
                "boots",
                "bc_valid",
                "bc_kind",
                "bc_line",
                "bc_col",
                "bc_pc",
                "bc_lr",
                "bc_boot",
            ] {
                assert_eq!(fields.next().unwrap().split_once('=').unwrap().0, key);
            }
            assert_eq!(fields.next(), None);
            let expected = match kind {
                0 => {
                    "PIPE_RESET v=1 resetreas=0xffffffff boots=4294967295 bc_valid=0 bc_kind=none bc_line=0 bc_col=0 bc_pc=0x00000000 bc_lr=0x00000000 bc_boot=0\n"
                }
                1 => {
                    "PIPE_RESET v=1 resetreas=0xffffffff boots=4294967295 bc_valid=1 bc_kind=panic bc_line=4294967295 bc_col=4294967295 bc_pc=0xffffffff bc_lr=0xffffffff bc_boot=4294967295\n"
                }
                _ => {
                    "PIPE_RESET v=1 resetreas=0xffffffff boots=4294967295 bc_valid=1 bc_kind=hardfault bc_line=4294967295 bc_col=4294967295 bc_pc=0xffffffff bc_lr=0xffffffff bc_boot=4294967295\n"
                }
            };
            assert_eq!(text, expected);
        }
    }

    #[test]
    fn retained_record_rejects_invalid_magic_kind_and_each_corrupt_word() {
        for kind in [1, 2] {
            let breadcrumb = Breadcrumb {
                kind,
                file_hash: file_hash("src/main.rs"),
                line: 123,
                column: 4,
                pc: u32::MAX,
                lr: 0x12345678,
                boot: 42,
            };
            let words = breadcrumb.encode();
            assert_eq!(Breadcrumb::decode(words), Some(breadcrumb));
            for index in 0..RECORD_WORDS {
                let mut corrupt = words;
                corrupt[index] ^= 1;
                assert_eq!(Breadcrumb::decode(corrupt), None);
            }
        }
        assert_eq!(Breadcrumb::decode([0; RECORD_WORDS]), None);
        assert_eq!(Breadcrumb::decode(Breadcrumb::default().encode()), None);
        assert_eq!(next_boot(0, 100), 1);
        assert_eq!(next_boot(COUNTER_MAGIC, 100), 101);
        assert_eq!(next_boot(COUNTER_MAGIC, u32::MAX), 0);
        assert_eq!(file_hash(""), 0x811c_9dc5);
        assert_eq!(file_hash("a"), 0xe40c_292c);
        assert_ne!(file_hash("src/main.rs"), file_hash("src/other.rs"));
    }
}
