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
`9b3376b11009f66f74fd34a01f65635a117dd591`. Establish that tag on this exact
ancestor before enabling publication. Do not tag the current S31 source as
`v0.1.0`: the ancestor baseline makes the existing `feat:` commits select
`v0.2.0-alpha.1` for the first generated ESP32-S31 release on `next`.
Confirm this with a semantic-release dry run before publishing; newer breaking
commits can change the expected version. Bootstrap tags and publication require
explicit maintainer action; installing this tooling creates neither.

The historical bundle can be built independently, without tagging or publishing:

```sh
python3 tools/release/build.py --version 0.1.0 \
  --source-sha 9b3376b11009f66f74fd34a01f65635a117dd591 \
  --profile esp32s3 --esp-image 'espressif/idf@sha256:REPLACE_WITH_VERIFIED_IDF_5_5_2_DIGEST' \
  --output-dir dist/legacy
```

Resolve and verify the historical ESP-IDF 5.5.2 image digest first; the placeholder
is intentionally invalid. The builder obtains the historical pinned NCS image
from that commit. This is an experimental S3/nRF pair: ESP32-S3 has **no native
Classic Bluetooth** and the pair is **not compatible with S31 LC3 firmware**.
Do not mix its nRF firmware with an S31 release.

## Published Bundles

Each release contains five required assets (the current builder names the UF2
`nrf52840`, not `xiao-nrf52840`):

- `omi-VERSION-esp32s31.zip`: ESP application, bootloader, partition table,
  generated flash metadata and matching flashing instructions.
- `omi-VERSION-nrf52840.uf2`: paired XIAO nRF52840 application firmware.
- `omi-VERSION-debug.zip`: ELF files and dependency/build provenance.
- `manifest.json`: firmware version, full source commit, protocol versions,
  pinned builder images, validation metadata and asset hashes/sizes.
- `SHA256SUMS`: checksums for the other four files, including the manifest.

Download both MCU images from the same release and verify `SHA256SUMS` before
flashing. Follow the ESP archive's generated README: flash offsets come from
the build's `flasher_args.json`, not hard-coded guesses. S31 instructions include
native USB JTAG/OpenOCD. Copy the UF2 to the XIAO bootloader drive; do not erase
or replace the nRF bootloader. The debug archive is not a flash bundle.

Firmware SemVer is separate from the mesh and SPI bridge protocol versions.
An alpha suffix does not establish wire compatibility; check the manifest and
use the paired images. The debug archive captures the resolved ESP component
lock, SDK configuration, frozen West manifest, resolved revisions, IDF commit
and version, and CMake metadata. Pinned containers and archived source provide
traceability, not a claim that rebuilding historical firmware is bit-identical.

## Publication Lifecycle

The release job requires the ESP-IDF, Python/shared-C and nRF verification jobs,
including the existing host sanitizer gates. Ordinary S31 CI builds remain
normal verification builds. Release `prepare` instead invokes the Docker-based
builder with the LC3 release profile against an archive of the exact source SHA:

```sh
python3 tools/release/build.py --version VERSION --source-sha FULL_COMMIT_SHA \
  --profile esp32s31 --output-dir dist/release
```

The output directory must be absent or empty. The builder requires and validates
all five assets, flash inputs, firmware identity, UF2 bounds, and protocol
metadata before returning. Preparation runs **before** semantic-release creates
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
