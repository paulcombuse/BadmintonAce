Build and upload the SPIFFS image with PlatformIO to install the web page, CSS,
logo, and localization catalog.

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

The browser and touchscreen support English and German. Change the **Language**
selector in the web interface; the selection is stored in ESP32 NVS and applies
to the web UI, status messages, and touchscreen labels. The shared translations
are in `locales.json` in this directory, so include it when building and
uploading the SPIFFS filesystem image. Missing browser translations fall back
to English; firmware status messages also have compiled English fallbacks.
