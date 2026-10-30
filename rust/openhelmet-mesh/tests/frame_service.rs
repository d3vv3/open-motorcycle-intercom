use openhelmet_mesh::{frame_service::*, tx_slot::Slot};
use std::cell::Cell;
fn config() -> Config {
    Config {
        voices: 8,
        interval: 10,
        frame: 20000,
        slot: 2000,
        control: 4000,
        guard: 500,
    }
}
#[test]
fn balanced_critical_regions_and_failure_short_circuits() {
    // Invalid/out-of-domain generation observations fail comparisons closed,
    // including a changed value on the second (atomic arm-publication) read.
    for (first, second, active, stopping, arm) in [
        (7, 7, 1, 0, 0),
        (8, 7, 1, 0, 0),
        (-1, 7, 1, 0, 0),
        (7, -1, 1, 0, 0),
        (7, 8, 1, 0, 0),
        (7, 7, 0, 0, 0),
        (7, 7, 1, 1, 0),
        (7, 7, 1, 0, -1),
        (7, 7, 1, 0, 42),
    ] {
        let held = Cell::new(false);
        let mut trace = Vec::new();
        let mut reads = 0;
        let mut owner = Slot::new();
        service(
            &mut |op, _, p| {
                trace.push(op);
                match op {
                    ENTER => {
                        assert!(!held.replace(true));
                        0
                    }
                    EXIT => {
                        assert!(held.replace(false));
                        0
                    }
                    GENERATION => {
                        assert!(held.get());
                        reads += 1;
                        if reads == 1 { first } else { second }
                    }
                    READ => {
                        assert!(held.get());
                        unsafe {
                            p.cast::<State>().write(State {
                                start: 100000,
                                counter: 10,
                            });
                        }
                        0
                    }
                    WRITE | ARM_WRITE => {
                        assert!(held.get());
                        0
                    }
                    _ => {
                        assert!(!held.get());
                        match op {
                            ACTIVE => active,
                            STOPPING => stopping,
                            CLOCK => 120000,
                            ARM => arm,
                            INDEX => -1,
                            _ => 0,
                        }
                    }
                }
            },
            |a| {
                assert!(!held.get());
                a.apply(&mut owner)
            },
            120000,
            7,
            config(),
        );
        assert!(!held.get());
        let admitted = first == 7 && second == 7 && active != 0 && stopping == 0;
        assert_eq!(trace.contains(&ARM), admitted);
        assert_eq!(trace.contains(&TICK), admitted && arm == 0);
        assert_eq!(trace.contains(&LOG), admitted && arm != 0 && arm != -1);
        if admitted && arm == 0 {
            let at = |op| trace.iter().position(|x| *x == op).unwrap();
            assert!(at(TICK) < at(COORDINATOR) && at(TICK) < at(INDEX));
        }
    }
    let mut c = config();
    c.interval = 0;
    service(
        &mut |_, _, _| panic!("invalid configuration invoked effect"),
        |_| panic!(),
        0,
        0,
        c,
    );
}
#[test]
fn dto_layouts_match_platform_header() {
    use std::mem::{align_of, offset_of, size_of};
    assert_eq!(offset_of!(State, counter), 8);
    assert_eq!(offset_of!(Arm, generation), 8);
    assert_eq!(offset_of!(Tick, now_ms), 4);
    assert_eq!(size_of::<Tick>(), 8);
    assert_eq!(offset_of!(Control, generation), 16);
    assert_eq!(size_of::<Control>(), 24);
    assert!(align_of::<State>() >= align_of::<i64>());
}
