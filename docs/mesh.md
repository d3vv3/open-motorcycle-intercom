# Mesh

Mesh lets nearby OMI devices talk to each other without a phone or internet connection.

## Get started

1. Hold the center **play** button for **2-5 seconds**, then release to turn mesh on.
2. Choose the same color channel on every device in your group. Green is the default.
3. Speak normally. Voice activation sends speech automatically; there is no push-to-talk button.

Tap **- / +** to adjust mesh volume. See [Buttons](buttons.md) for all controls, including how to turn mesh off.

## Channels

| Channel | Indicator |
| --- | --- |
| Green (default) | Green LED |
| Red | Red LED |
| Blue | Blue LED |

The LED shows the selected channel, which is saved across restarts.
To change channels, **pause Bluetooth music**, then hold **- / +** for **1 second** and release.
While Bluetooth music is playing, the same holds skip songs instead.

## How it works

One device automatically coordinates the group; you do not need to pick it.
The firmware can rearrange the group as links change.

A device between two others can relay their voices: A to B to C is the limit of **two radio hops**, with one relay, not an unlimited chain.
Relaying adds delay. Separate incoming voices play independently rather than waiting for each other.

The design supports up to **eight devices** and **two simultaneous talkers**.
The start of speech may be clipped, and connection changes may interrupt audio.
