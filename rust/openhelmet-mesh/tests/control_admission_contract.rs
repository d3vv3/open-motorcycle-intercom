use openhelmet_mesh::control_admission::{Admission, Control, classify};

#[test]
fn exhaustive_control_discriminator_and_envelope() {
    // Independent wire contract, including all three shared-shape handovers.
    let rows = [
        (6, 14),
        (9, 5),
        (10, 3),
        (14, 47),
        (15, 15),
        (16, 15),
        (17, 20),
        (18, 25),
        (19, 11),
        (20, 25),
        (21, 25),
        (22, 79),
        (23, 13),
    ];
    for kind in 0..=255 {
        let expected = rows.iter().find(|row| row.0 == kind).map(|row| row.1);
        assert_eq!(
            Control::from_wire(kind).map(Control::expected_size),
            expected
        );
        for flags in 0..=255 {
            for ttl in 0..=3 {
                for size in [
                    0,
                    expected.unwrap_or(1) - 1,
                    expected.unwrap_or(1),
                    expected.unwrap_or(1) + 1,
                    u16::MAX,
                ] {
                    let result = classify(kind, size, flags, ttl);
                    let Some(expected) = expected else {
                        assert_eq!(result, Admission::Unknown);
                        continue;
                    };
                    if size == expected && (flags == 0 && ttl == 2 || flags == 2 && ttl == 1) {
                        assert_eq!(
                            result,
                            Admission::Envelope(Control::from_wire(kind).unwrap())
                        );
                    } else {
                        assert_eq!(result, Admission::Rejected);
                    }
                }
            }
        }
    }
}
