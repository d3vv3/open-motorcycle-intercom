use openhelmet_mesh::control_window::*;
#[test]
fn admission_selection_and_frozen_continuation() {
    assert_eq!(core::mem::size_of::<Context>(), 32);
    assert_eq!(core::mem::offset_of!(Context, role), 28);
    assert!(window_open(100, 200, 100));
    assert!(window_open(100, 200, 200));
    assert!(!window_open(100, 200, 99));
    assert!(!window_open(100, 200, 201));
    assert!(window_open(i64::MAX - 10, i64::MIN + 10, i64::MIN));
    assert_eq!(select(10, true, -1, 8, 10, false), SYNC);
    assert_eq!(select(1, false, 1, 8, 10, false), QUEUED);
    assert_eq!(select(10, false, 2, 8, 10, true), REJECT);
    assert_eq!(select(16, true, 0, 8, 10, true), SYNC);
    assert!(grace(true, true, 1, 1, 100, 100, 16, 8));
    assert!(!grace(true, true, 1, 1, 101, 100, 16, 8));
    assert!(!grace(true, true, 1, 1, 0, u32::MAX, 16, 8));
    assert!(!grace(true, true, 1, 1, 100, 100, 8, 8));
    assert!(!grace(false, true, 1, 1, 100, 100, 16, 8));
    assert!(!grace(true, false, 1, 1, 100, 100, 16, 8));
    assert!(!grace(true, true, 2, 1, 100, 100, 16, 8));
    assert!(grace(true, true, 1, 1, u32::MAX, 0, 16, 8));
    let original = Context {
        control_generation: 7,
        tdma_generation: 7,
        lifecycle: 3,
        session: 9,
        frame: 1,
        term: 1,
        local: 1,
        coordinator: 1,
        owner_local: 1,
        leader: 1,
        role: 2,
        slot: 1,
        active: true,
        stopping: false,
    };
    assert!(continuation(&original, &original));
    for field in 0..14 {
        let mut current = original;
        match field {
            0 => {
                current.control_generation += 1;
                current.tdma_generation += 1;
            }
            1 => current.tdma_generation += 1,
            2 => current.lifecycle += 1,
            3 => current.session += 1,
            4 => current.frame += 1,
            5 => current.role += 1,
            6 => current.slot += 1,
            7 => current.active = false,
            8 => current.stopping = true,
            9 => current.term += 1,
            10 => current.local += 1,
            11 => current.coordinator += 1,
            12 => current.owner_local += 1,
            _ => current.leader += 1,
        }
        assert!(!continuation(&original, &current));
    }
}
