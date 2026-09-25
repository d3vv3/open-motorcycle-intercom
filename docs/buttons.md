# BOOT Button Gestures

Use BOOT only after the firmware has started:

- Release after 50 ms to under 2 seconds: contextual call or A2DP play/pause
  control. Releases under 50 ms have no action.
- Hold for 2 to under 6 seconds, then release: toggle mesh.
- Hold for at least 6 seconds, then release: open Bluetooth pairing for 120 seconds.

Mesh enabled uses a rising two-note beep. Mesh disabled uses a falling two-note
beep. Bluetooth pairing uses three high beeps.

Do not hold BOOT while pressing RESET or applying power. That enters firmware
download mode instead of registering a runtime gesture.
