# BadmintonAce
This is the revitalization repository by Eike Dohmen of the CAD, Code and Documentation for Badminton ACE.
It is based on the repository created by Peter Sinclair for his Hackaday Prize 2023 Entry
https://hackaday.io/project/190336-badminton-ace-shuttle-launcher

## Project scope
Badminton Ace is a project to create a relatively low cost multi-axis robot capable of launching badminton shuttles in user defined patterns for teaching and training. This project seeks to engage children and adults in the exciting sport of badminton to improve physical health while providing an engaging mechatronics system that demonstrates the power of STEM.

Users or trainers can connect to the launcher using wifi then select and adjust shot placement and pattern to learn or practice new badminton return techniques. This will be done directly in the browser using an interactive 3D court with controls for rotation and trajectory.
For kids to use Badminton ACE on their own and get them away from SmartPhones a touch TFT was added, enabling to load different presets.

## Status LED
The firmware uses a 10-LED WS2812B-compatible strip on `statusLedPin` (GPIO13), configured by `statusLedCount` in `src/ACE.cpp`. All LEDs show the same color and brightness at any given time. Set `statusLedCount` to match the number of LEDs in the connected strip. Power the strip from a suitably rated external 5 V supply, with a common ground to the ESP32; do not power it from an ESP32 GPIO or 3.3 V rail. Use a 3.3 V-to-5 V logic-level shifter on the data line, a series data resistor, and bulk capacitance near the strip.

The indication follows machine and obstruction state: blue while the ultrasonic sensor reports a persistent obstruction; red for two seconds after an obstruction clears or the launcher stops; a red-to-green pulse countdown before a run; green while running; and delayed, dim red breathing while idle. Obstruction indication has priority over all other states. Idle breathing is capped between brightness values 4 and 28 (on a 0–255 scale), with an overall LED brightness cap of 64.

## Revitalization
Additionally many tiny adjustments where made in the CAD files, to improve assembly and operation. An alternate gripper+puller concept for gripping the next shuttle with a moving gripper servo mounted to a linear slider carriage was implemented. This is just a (temporary) enhancement to investigate if it outperforms the static gripper+pusher concept, though staying fully compatible with the original concept. Additionally a first draft for the realization of a status light within the tube base was generated but needs a redesign with focus on manufacture by Fused Filament Deposition (FFF).

## Education
The robot can also be used as a powerful STEM educational tool throughout the build process if desired. With multiple sensors, 2 brushless and 4 servo motors, 3D printing, laser cutting, urethane molding, programing, inverse kinematics, and more, this project covers a wide range of topics for those looking to learn how it all works. It also helps to encourage excercise for it's users.
