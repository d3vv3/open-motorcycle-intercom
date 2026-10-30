use openhelmet_mesh::sync_correction::{correction, next_delay};
#[test]
fn literal_correction_boundaries() {
    for (actual, drift, error) in [
        (100000, 0, false),
        (100100, 100, false),
        (99899, -101, true),
        (100101, 101, true),
        (99500, -500, true),
        (99499, -500, true),
        (100500, 500, true),
        (100501, 500, true),
        (4295067296, 0, false),
    ] {
        let p = correction(100000, 0, 0, actual + 100, 0, 100, 20000);
        assert_eq!(
            (p.actual_start_us, p.drift_us, p.error),
            (actual, drift, error)
        );
    }
    for (local, frame, start) in [
        (u32::MAX, 0, 120000),
        (0, u32::MAX, 80000),
        (0, 107375, 2147600000),
        (0, 0x80000000, -42949672860000),
        (0, 0x7fffffff, 42949673040000),
    ] {
        let p = correction(100000, local, frame, start + 100, 0, 100, 20000);
        assert_eq!((p.actual_start_us, p.drift_us), (start, 0));
    }
    assert_eq!(
        correction(i64::MAX, 0, 0, i64::MIN, 0, 0, 20000).drift_us,
        1
    );
    assert_eq!(
        correction(0, 0, 0, i64::MIN, 1, 0, 20000).actual_start_us,
        i64::MAX
    );
}
#[test]
fn fresh_delay_is_normalized_and_exact_boundary_is_full_frame() {
    for (now, start, delay) in [
        (100000, 100000, 20000),
        (140000, 100000, 20000),
        (140001, 100000, 19999),
        (99999, 100000, 1),
        (79999, 100000, 1),
        (i64::MIN, i64::MAX, 19999),
    ] {
        assert_eq!(next_delay(now, start, 20000), delay);
    }
}
