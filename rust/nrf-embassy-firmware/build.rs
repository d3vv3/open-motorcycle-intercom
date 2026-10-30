use std::{env, fs, path::Path, process::Command};

fn git(manifest: &Path, args: &[&str]) -> Option<String> {
    let output = Command::new("git")
        .arg("-C")
        .arg(manifest)
        .args(args)
        .output()
        .ok()?;
    output
        .status
        .success()
        .then(|| String::from_utf8(output.stdout).ok())
        .flatten()
        .map(|text| text.trim().to_owned())
}

fn main() {
    let manifest = env::var("CARGO_MANIFEST_DIR").unwrap();
    let manifest = Path::new(&manifest);
    let out = env::var("OUT_DIR").unwrap();
    let out = Path::new(&out);
    println!("cargo:rerun-if-changed=build.rs");
    // The release packager builds from a git archive (no HEAD) and names the version/SHA itself.
    for name in ["OMI_FIRMWARE_VERSION", "OMI_GIT_SHA"] {
        println!("cargo:rerun-if-env-changed={name}");
    }
    let version =
        env::var("OMI_FIRMWARE_VERSION").unwrap_or_else(|_| env::var("CARGO_PKG_VERSION").unwrap());
    let sha = env::var("OMI_GIT_SHA")
        .ok()
        .or_else(|| git(manifest, &["rev-parse", "--verify", "HEAD"]))
        .unwrap_or_else(|| "unavailable-no-git-head".to_owned());
    for name in ["HEAD", "packed-refs"] {
        if let Some(path) = git(
            manifest,
            &["rev-parse", "--path-format=absolute", "--git-path", name],
        ) {
            println!("cargo:rerun-if-changed={path}");
        }
    }
    if let Some(reference) = git(manifest, &["symbolic-ref", "-q", "HEAD"])
        && let Some(path) = git(
            manifest,
            &[
                "rev-parse",
                "--path-format=absolute",
                "--git-path",
                &reference,
            ],
        )
    {
        println!("cargo:rerun-if-changed={path}");
    }
    let mut metadata = format!(
        "identity=bridge-runtime-integration\nversion={}\nsource_sha={}\nsource_state=unqualified-working-tree\nqualification=experimental-not-production-not-rf-qualified\nbackend=live-strict-full-protocol-hardware-unqualified\ncapability=strict-init-external-ingress-published-gate-not-rf-qualification\nradio=actor-owned-activate-stopped\nrf_tx=physical-ticket-wait-pipe0-e7e7e7e7e7\nrf_profile=dpl-2mbps-crc16-noack-channels40-20-60-pipes03\nrf_irq_priority=radio-timer2-timer4-p2-timer0-p3\nrx_validation=pending-delegated-worker\ndeadline_guard=live-same-facade-unqualified\ndeadline_hardware=timer4-1mhz-32bit-cc0-cutoff-cc1-start-cc3-capture\ndeadline_routes=ppi3-radio-disable-group0-dis-ppi4-radio-txen\ndeadline_validation=pending-delegated-worker\nrf_timing=unqualified-no-tx-bench\nusb=cdc-diagnostics-linked-unqualified\ntdma=actor-owned-timer0-checked-window-callbacks\nws=actor-owned-tdma-frame-capture\nclock_origin=shared-timer0\nruntime_clock=shared-timer0\ningress=single-16-mailbox-direct-external-consume\nbridge_rpc=live-single-rendezvous-actual-mutation-results\nasync_wake=actor-signals-rtc1-200us-fallback\nrx_stamp=irq-handling-after-quiescence\n",
        version, sha,
    );
    if env::var_os("CARGO_FEATURE_EARLY_TIMING_SERVICE").is_some() {
        metadata.push_str("early_timing_service=EarlyTimingServiceExperimental\n");
    }
    if env::var_os("CARGO_FEATURE_SAME_SLOT_AUDIO_REFIT").is_some() {
        metadata.push_str("same_slot_audio_refit=SameSlotAudioRefitExperimental\n");
    }
    if env::var_os("CARGO_FEATURE_NONBLOCKING_BRIDGE_DELIVERY").is_some() {
        metadata.push_str("nonblocking_bridge_delivery=NonblockingBridgeDeliveryExperimental\n");
    }
    if env::var_os("CARGO_FEATURE_UNMASKED_BRIDGE_OWNER").is_some() {
        metadata.push_str("unmasked_bridge_owner=UnmaskedBridgeOwnerExperimental\n");
    }
    if env::var_os("CARGO_FEATURE_ADMISSION_HEADROOM").is_some() {
        metadata.push_str("admission_headroom=AdmissionHeadroomExperimental\n");
    }
    fs::write(out.join("integration-metadata.txt"), metadata).unwrap();
    // The revision identifies source HEAD, not the bytes of this dirty image.
    let identity = format!(
        "identity=bridge-runtime-integration version={} source_HEAD={} source_state=unqualified-working-tree backend=live-strict-full-protocol-hardware-unqualified bridge=6 mesh=5 RFqualified=false radio=actor-owned-unqualified clock_origin=shared-timer0\n",
        version, sha,
    );
    assert!(identity.is_ascii() && identity.len() <= 512);
    fs::write(out.join("usb-identity.txt"), identity).unwrap();
    // Host render tests use exactly the generated firmware identity.
    if env::var("CARGO_CFG_TARGET_ARCH").unwrap() != "arm" {
        return;
    }
    for file in ["memory.x", "integration.x"] {
        fs::copy(manifest.join(file), out.join(file)).unwrap();
        println!("cargo:rerun-if-changed={file}");
    }
    println!("cargo:rustc-link-search={}", out.display());
    let flags = env::var("CARGO_ENCODED_RUSTFLAGS").unwrap_or_default();
    if !flags
        .split('\u{1f}')
        .any(|flag| flag == "link-arg=-Tlink.x")
    {
        println!("cargo:rustc-link-arg-bin=nrf-embassy-firmware=-Tlink.x");
    }
    println!("cargo:rustc-link-arg-bin=nrf-embassy-firmware=-Tintegration.x");
    println!("cargo:rustc-link-arg-bin=nrf-embassy-firmware=--nmagic");
}
