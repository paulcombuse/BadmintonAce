Upload the web page, CSS file, and logo graphic to SPIFFS with PlatformIO's
Build Filesystem Image and Upload Filesystem Image actions.

The custom user preset is saved separately in ESP32 NVS, so it survives
reboots and SPIFFS filesystem uploads. In the web interface, select and edit
shots, build the sequence, choose a play mode, enter a name, and select
**Save Current Pattern**. The saved preset appears in the preset list and can
be started directly with the touchscreen's **Custom User Preset** button.

The firmware supports up to 10 shots and 50 sequence entries. `Play Once` and
`Repeat` traverse the sequence in order; `Mix Order` randomly chooses a
sequence entry for every shot (so entries can repeat); `Random Shots` chooses
from all saved shots and ignores the sequence. The touchscreen runs the saved
mode, sequence, and delays.
