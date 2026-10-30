//! Replacement requirement for topology_legacy_received's private counters.
//! Compile an unchanged shipping topology module with descendant assertions:
//! counters intentionally have no public ABI getter and do not drive report PDR.
use std::{fs, process::Command};
mod support;
use support::TempDir;

const REQUIREMENT: &str = r#"
#[cfg(test)]
mod legacy_audio_requirement {
    #[test]
    fn old_duplicate_gap_and_wrap_do_not_inflate_evidence() {
        let mut t = super::Topology::default();
        t.audio(2, 100);
        assert_eq!((t.neighbors[1].received, t.neighbors[1].lost), (1, 0));
        for seq in [90, 100] {
            t.audio(2, seq);
            assert_eq!((t.neighbors[1].received, t.neighbors[1].lost), (1, 0));
        }
        t.audio(2, 101);
        t.audio(2, 104);
        assert_eq!((t.neighbors[1].received, t.neighbors[1].lost), (3, 2));
        // Rejected old values must not rewind the accepted sequence cursor.
        for seq in [90, 104, 103, 104] { t.audio(2, seq); }
        assert_eq!((t.neighbors[1].received, t.neighbors[1].lost), (3, 2));
        t.clear(2);
        for seq in [254, 255, 0, 3, 3, 255, 4] { t.audio(2, seq); }
        assert_eq!((t.neighbors[1].received, t.neighbors[1].lost), (5, 2));
        assert_eq!((t.neighbors[0].received, t.neighbors[0].lost), (0, 0));
        t.reinit();
        t.audio(2, 90);
        assert_eq!((t.neighbors[1].received, t.neighbors[1].lost), (1, 0));
    }
}
"#;

#[test]
fn shipping_topology_audio_replaces_legacy_counter_requirement() {
    let temp = TempDir::new();
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let source = fs::read_to_string(root.join("rust/openhelmet-mesh/src/topology.rs")).unwrap();
    let module = temp.0.join("topology.rs");
    let wrapper = temp.0.join("requirements.rs");
    fs::write(
        &wrapper,
        format!("pub use openhelmet_mesh::{{adaptive, core}};\n#[path={module:?}] mod topology;"),
    )
    .unwrap();
    let library = temp.0.join("libopenhelmet_mesh.rlib");
    assert!(
        Command::new("rustc")
            .args([
                "--edition=2021",
                "--crate-name=openhelmet_mesh",
                "--crate-type=rlib",
            ])
            .arg(root.join("rust/openhelmet-mesh/src/lib.rs"))
            .arg("-o")
            .arg(&library)
            .status()
            .unwrap()
            .success()
    );
    let executable = temp.0.join("requirements");
    for (name, from, to) in [
        ("shipping", "", ""),
        (
            "old-counted",
            "crate::core::SeqClass::First\n",
            "crate::core::SeqClass::OldReset | crate::core::SeqClass::First\n",
        ),
        (
            "duplicate-counted",
            "crate::core::SeqClass::First\n",
            "crate::core::SeqClass::Duplicate | crate::core::SeqClass::First\n",
        ),
        (
            "gap-not-counted",
            "n.lost = n.lost.wrapping_add(u32::from(result.gap));",
            "n.lost = n.lost.wrapping_add(0);",
        ),
    ] {
        let mutated = if name == "shipping" {
            source.clone()
        } else {
            assert_eq!(source.matches(from).count(), 1, "mutation anchor: {name}");
            source.replace(from, to)
        };
        fs::write(&module, format!("{mutated}\n{REQUIREMENT}")).unwrap();
        let build = Command::new("rustc")
            .args(["--edition=2021", "--test", "--extern"])
            .arg(format!("openhelmet_mesh={}", library.display()))
            .arg(&wrapper)
            .arg("-o")
            .arg(&executable)
            .output()
            .unwrap();
        assert!(
            build.status.success(),
            "{name}: {}",
            String::from_utf8_lossy(&build.stderr)
        );
        let output = Command::new(&executable)
            .arg("legacy_audio_requirement::")
            .output()
            .unwrap();
        let stdout = String::from_utf8_lossy(&output.stdout);
        if name == "shipping" {
            assert!(output.status.success(), "{stdout}");
        } else {
            assert_eq!(output.status.code(), Some(101), "{name}: {output:?}");
            assert!(
                stdout.contains("assertion `left == right` failed"),
                "{name}: {stdout}"
            );
        }
    }
}
