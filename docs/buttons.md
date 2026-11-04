# Buttons

Use the **-**, **play**, and **+** buttons. For setup, see [button wiring](wiring.md#external-buttons).
Hold for the time shown, then release to perform the action.

| Action | Controls |
| --- | --- |
| Mesh volume (Bluetooth call volume during an active or held call) | Tap **- / +** |
| Bluetooth volume (music and calls) | Hold **play** and tap **- / +** |
| Play or pause music | Tap **play** |
| Answer or end a call | Tap **play** |
| Previous or next song | While Bluetooth music is playing, hold **- / +** for **1 second** |
| Previous or next mesh channel | Pause Bluetooth music, then hold **- / +** for **1 second** |
| Turn mesh on or off | Hold **play** for **2-5 seconds** |
| Pair a phone | Hold **play** for **6 seconds**, then connect within **2 minutes** |

- Three beeps mean minimum or maximum volume. Volume levels reset when you restart the device.
- Channels cycle **green -> red -> blue**. The LED shows your channel, which is saved across restarts.
- Use the same channel as the people you want to talk to.
- After adjusting Bluetooth volume, releasing **play** does not trigger another action.
- Bluetooth volume also adjusts the paired phone's volume when supported; music and calls keep separate levels, and call volume moves in coarser steps.
- Incoming calls announce once; ringing does not mute the mesh. Answering or holding a call mutes only this device's mesh microphone and local mesh playback. Other mesh participants remain connected and can relay through this device. Ending the call restores local mesh audio after the transport confirms the flush.

## Hold Prompts

While you hold a button, the device says what releasing it now would do:

- **play** at **2 seconds**: "Mesh On" or "Mesh Off".
- **play** at **6 seconds**: "Phone Pairing".
- **- / +** at **1 second**: the channel the release would select. While Bluetooth music is playing, the hold skips songs instead and says nothing.

Releasing does not repeat the prompt. Follow-up prompts such as "Bluetooth off" still play.

## Phone Sounds

- A short high beep when the phone is fully connected (music and calls).
- A lower beep when the phone fully disconnects. There is no beep when the device turned Bluetooth off itself.
- "Phone paired" after pairing a new phone.
- Connect and disconnect beeps are skipped during a ringing or active call.

## ESP-NOW Mesh and Bluetooth

Without the nRF52840, mesh and Bluetooth share the S31 radio, so only one is on at a time:

- Turning mesh on turns Bluetooth off and says "Bluetooth off". The phone disconnects and cannot reconnect while mesh is on.
- Turning mesh off turns Bluetooth back on, says "Bluetooth on", and tries once to reconnect the phone that mesh on disconnected.
- Holding **play** to pair turns mesh off ("Mesh Off"), then opens pairing.
- At startup, the saved mesh setting decides: Bluetooth stays off if mesh was left on.

With the nRF52840 transport, mesh and Bluetooth run together.

The onboard **BOOT** button is for firmware flashing, not everyday controls.
