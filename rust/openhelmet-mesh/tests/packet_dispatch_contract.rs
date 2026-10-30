use openhelmet_mesh::packet_dispatch::{Dispatch as D, dispatch};
#[test]
fn literal_family_matrix() {
    for kind in 0..=255 {
        for size in [0, 1, 2, 3, 7, 8, 9, 255] {
            for bound in [false, true] {
                let expected = match kind {
                    1 => D::Audio,
                    6 => D::SlotMap,
                    9 => D::Grant,
                    10 => D::Release,
                    7 if bound && size == 8 => D::Status,
                    8 if bound && size == 2 => D::Keepalive,
                    4 if bound => D::Leave,
                    4 | 7 | 8 => D::Rejected,
                    _ => D::Unknown,
                };
                assert_eq!(dispatch(kind, size, bound), expected);
            }
        }
    }
}
