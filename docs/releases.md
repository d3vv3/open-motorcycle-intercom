# Firmware Releases

## Channels and Versions

`next` is the intended default development branch and publishes alpha releases
after successful push verification. The semantic-release configuration uses
`{ "name": "next", "prerelease": "alpha" }`; tags are `v${version}`.
Changing the GitHub default branch is a separate administrator action, not done
by this configuration.

`main` is the stable branch. Pushes to `main` run verification only, never release.
When ready, merge `next` into `main`, then manually run the **Build** workflow
with **Run workflow** on `main`. With the historical `v0.1.0` baseline and the
current feature history, the first stable release is expected to be `v0.2.0`.
Manual dispatch on `next` can also publish an alpha. Pull requests and all other
branches cannot publish. A manual run still needs a releasable commit; it does
not force a new version.

Conventional Commits determine versions relative to the last reachable release:

| Commit | Release increment |
| --- | --- |
| `fix: ...`, `perf: ...` | Patch |
| `feat: ...` | Minor |
| `feat!: ...`, any type with `!`, or a `BREAKING CHANGE:` footer | Major |
| `docs: ...`, `test: ...`, `chore: ...` without breaking changes | None |

Both analysis and release notes explicitly use the `conventionalcommits` preset.
Normal SemVer rules apply before 1.0 too: a breaking change from 0.x produces
1.0.0 (or 1.0.0-alpha.1 on `next`), not a special 0.x minor bump. Alpha sequence
numbers are managed automatically. Preserve Conventional Commit messages when
merging or squash-merging.

## Initial Bootstrap

The historical ESP32-S3 release is `v0.1.0`, pointing to
`9b3376b11009f66f74fd34a01f65635a117dd591`. That ancestor baseline made the
existing `feat:` commits select `v0.2.0-alpha.1` for the first generated
ESP32-S31 release on `next` (Zephyr nRF firmware). Releases after it pair the
ESP32-S31 C firmware with the Rust Embassy nRF52840 firmware; with no breaking
commits the next alpha is `v0.2.0-alpha.2`. Confirm the expected version with a
semantic-release dry run before publishing; newer breaking commits can change
it. Bootstrap tags and publication require explicit maintainer action;
installing this tooling creates neither.

The builder now supports only the `esp32s31` profile. The experimental legacy
`esp32s3` profile (the `v0.1.0` S3/Zephyr pair, built with NCS) was retired with
the Zephyr nRF application; rebuild that bundle with `tools/release/build.py`
from the `v0.2.0-alpha.1` tag if it is ever needed. Never mix nRF and ESP images
from different releases.

## Published Bundles

Each release contains five required assets (the current builder names the UF2
`nrf52840`, not `xiao-nrf52840`):

- `omi-VERSION-esp32s31.zip`: ESP application, bootloader, partition table,
  generated flash metadata and matching flashing instructions.
- `omi-VERSION-nrf52840.uf2`: paired XIAO nRF52840 application firmware (the
  Rust Embassy `rust/nrf-embassy-firmware` crate, built with cargo; the debug
  archive keeps the historical `nrf/zephyr.elf` member name).
- `omi-VERSION-debug.zip`: ELF files and dependency/build provenance.
- `manifest.json`: firmware version, full source commit, protocol versions,
  pinned ESP-IDF builder image, the Rust nRF build (`nrf_build`: cargo argv,
  target and `rustc` version), validation metadata and asset hashes/sizes.
- `SHA256SUMS`: checksums for the other four files, including the manifest.

Download both MCU images from the same release and verify `SHA256SUMS` before
flashing. Follow the ESP archive's generated README: flash offsets come from
the build's `flasher_args.json`, not hard-coded guesses. S31 instructions include
native USB JTAG/OpenOCD. Copy the UF2 to the XIAO bootloader drive; do not erase
or replace the nRF bootloader. The debug archive is not a flash bundle.

Firmware SemVer is separate from the mesh and SPI bridge protocol versions.
An alpha suffix does not establish wire compatibility; check the manifest and
use the paired images. The debug archive captures the resolved ESP component
lock, SDK configuration and defaults, IDF commit and version, CMake metadata,
and the nRF `Cargo.lock`, cargo argv and `rustc -vV` output. Pinned containers and archived source provide
traceability, not a claim that rebuilding historical firmware is bit-identical.

## Publication Lifecycle

The release job requires the ESP-IDF and Python/shared-C jobs (including the host
sanitizer gates) and the whole Rust nRF workflow (`rust-nrf.yml`, called from
`build.yml`). Ordinary S31 CI builds remain normal verification builds. Release
`prepare` instead invokes the builder against an archive of the exact source
SHA. It builds the ESP32-S31 image in the digest-pinned ESP-IDF Docker image,
with the LC3 options, `sdkconfig.defaults` plus `sdkconfig.release.esp32s31.defaults`,
and `OMI_FIRMWARE_VERSION`/`OMI_GIT_SHA`. It builds the nRF52840 image on the
runner with `cargo build --locked --release --target thumbv7em-none-eabihf`
(Rust 1.95.0, installed by the release job) with the same version and SHA in
the environment for `build.rs`:

```sh
python3 tools/release/build.py --version VERSION --source-sha FULL_COMMIT_SHA \
  --profile esp32s31 --output-dir dist/release
```

The output directory must be absent or empty. The builder requires and validates
all five assets, flash inputs, firmware identity, UF2 bounds, and protocol
metadata before returning. Both images must embed the release version and full
source SHA. Preparation runs **before** semantic-release creates
the tag; validation failure therefore prevents tagging/publication. The GitHub
plugin uploads `dist/release/*`. A later network/upload failure can still leave
a tag or a partial GitHub release; inspect and repair it deliberately rather
than blindly rerunning or moving a published tag.

Release jobs share the `firmware-release` concurrency group, without cancelling
an in-progress publisher. Checkout uses the event's exact `github.sha` with full
history and tags. Remote branch-tip checks reject stale queued runs before
semantic-release and again after preparation; semantic-release also performs
its own branch/push checks. Do not advance a publishing branch during a release:
there is still a small race between the final check and remote tag creation.

Only the release job receives `contents: write`. It uses the built-in
`GITHUB_TOKEN`; issue/PR success and failure comments, labels, and discussions
are disabled, so no issue/PR write permissions are needed. This token does not
trigger downstream tag/release workflows. Building and asset publication are
therefore in this same workflow, not a separate tag-triggered build.

There is no npm publishing, package-version update, changelog commit, or bot
release commit. The private Node package only supplies pinned tooling. Use
Node.js 24 (at least 24.10.0) and `npm ci` with the committed lockfile. To resolve
an intentional tooling update, regenerate the lock with
`npm install --package-lock-only --ignore-scripts` and review it.

Before enabling publication, run the repository validation separately:

```sh
npm ci --ignore-scripts
uv run --frozen pytest
npx semantic-release --dry-run
```

Also parse/review the workflow YAML. A semantic-release dry run needs the
release branches, baseline tag, full Git history, and suitable repository
authentication; it skips `prepare`, so it does not validate firmware bundles.
Do not place tokens in command lines or commit them to the repository.
