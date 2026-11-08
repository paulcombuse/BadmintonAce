# BadmintonAce
This is the revitalization repository by Eike Dohmen of the CAD, Code and Documentation for Badminton ACE.
It is based on the repository created by Peter Sinclair for his Hackaday Prize 2023 Entry
https://hackaday.io/project/190336-badminton-ace-shuttle-launcher

## Project scope
Badminton Ace is a project to create a relatively low cost multi-axis robot capable of launching badminton shuttles in user defined patterns for teaching and training. This project seeks to engage children and adults in the exciting sport of badminton to improve physical health while providing an engaging mechatronics system that demonstrates the power of STEM.

Users or trainers can connect to the launcher using wifi then select and adjust shot placement and pattern to learn or practice new badminton return techniques. This will be done directly in the browser using an interactive 3D court with controls for rotation and trajectory.
For kids to use Badminton ACE on their own and get them away from SmartPhones a touch TFT was added, enabling to load different presets.

## Education
The robot can also be used as a powerful STEM educational tool throughout the build process if desired. With multiple sensors, 2 brushless and 4 servo motors, 3D printing, laser cutting, urethane molding, programing, inverse kinematics, and more, this project covers a wide range of topics for those looking to learn how it all works. It also helps to encourage excercise for it's users.

## Revitalization
I wanted to revitalize the projects started by Peter Sinclair and performed many tiny adjustments especially in the CAD files, to improve assembly and operation.
An alternate gripper+puller concept for gripping the next shuttle with a moving gripper servo mounted to a linear slider carriage was implemented. This is just a (temporary) enhancement to investigate if it outperforms the static gripper+pusher concept, though staying fully compatible with the original concept. Additionally a first draft for the realization of a status light within the tube base was generated but needs a redesign with focus on manufacture by Fused Filament Deposition (FFF).

## Touch TFT
A 4.0" TFT Module with 480x320 px and an SPI interface (using a `ST7796` integrated circuit) was added to the design. The touch functionality for the displays used is realized by a `XPT2046` integrated circuit. Please take care for matching or compatible driver ICs as there are many versions of these displays offered, but not all of them are performing well or are compatible with the utilized `TFT_eSPI` library.
The pins used on the ESP32 are:
* Data to TFT - TFT_MISO (GPIO19)
* Data from TFT - TFT_MOSI (GPIO23)
* Clock for TFT - TFT_SCLK (GPIO18)
* Chip select for TFT - TFT_CS (GPIO5)
* Data or command selection for TFT (high: command, low: data) - TFT_DC (GPIO26)
* Reset pin for TFT - TFT_RST (GPIO4)
* Backlight for TFT - TFT_BL (GPIO0)
* Chip select for touch - TOUCH_CS (GPIO15)

The Touch TFT used for design was bought here:
https://www.roboter-bausatz.de/p/4-tft-touchscreen-display-modul-st7796-480x320
Specifications of the Touch TFT can be found here:
https://www.lcdwiki.com/4.0inch_SPI_Module_ST7796#Product_Documentation

## Status LED
In order to better see what the machine does when standing opposite to it on the court - a status LED has been implemented utilizing a flexible `WS2812B` LED stripe which is placed inside the Tube Base part.
Currently all LEDs show the same color and brightness - though, WS2812 LEDs may be controlled individually. Set `statusLedPin` (default is GPIO13) and `statusLedCount` to match the number of LEDs of the connected strip. The strip gets powered with internal 5 V supplied by the Electronic Speed Controllers (ESCs).

The status LED indication follows machine and obstruction state:
* blue means the ultrasonic sensor reports a persistent obstruction,
* red for two seconds after an obstruction clears or the launcher stops,
* red-to-green pulses for countdown before a run,
* green means running,
* dimmed red breathing is the idle state.
Obstruction indication has priority over all other states. Idle breathing is capped between brightness values 4 and 28 (on a 0–255 scale), with an overall LED brightness cap of 64.

## Printed circuit board (PCB)
It is possible to realize a fully functional ACE with an arbitrary ESP32 board and without using the specialized PCB. Though, as a lot of people are feeling overwhelmed by electronics and microcontrollers but would love to realize such a machine for their kids or their club, I wanted to supply a plug-and-play PCB, which has proper annotations and by that should reduce the propablity to destroy components, reduce potential frustration and increase fun.
 ![PCB frontside](https://github.com/paulcombuse/BadmintonAce/blob/revitalization/documentation/img/PCB_Front.png)
 ![PCB backside](https://github.com/paulcombuse/BadmintonAce/blob/revitalization/documentation/img/PCB_Back.png)

 ## Shooting wheels
 The most challenging part of the machine in regards of interrelation of design, material specification and performance are the shooting wheels. The wheels necessitate to components: the inner rim which needs to withstand high mechanical stresses resulting from rotational speeds of up to 5000 rpm (in theory up to 8000 rpm) and on the other hand we need a flexible tyre with high surface friction in order to ensure "gripping" of each shuttle, squeeze prevention of the cork of each shuttle and maximum energy transfer by following the shuttle heads shape during acceleration of it.
 If the tyre is too "stiff" the mechanical stresses within our ACE motor mount and the motor's bearings will be high, but the shuttles will hardly fly 2 m as most of the energy is transfered to our design but not to the shuttle.
 So, ideally - referring to Baddy findings - the stiffness of the "rubber" tyre should be around `45 Shore A` hardness. To realize this flexibility while making the tyre withstand thousands of shots in terms of wear resistance, polyurethane is the material to choose.
 Peter and the Baddy community use moulded tyres, which needs a mould, preferably a degassing unit and something to balance the final wheel assembly.
 I wanted to use a different approach to make things more reproducible and easier: 3D printing of the wheels as a two component assemply print, utilizing PETG for the rim and `soft` TPU for the tyre. In order to get there I redesigned the wheel assembly to increase design compliance by increasing the radial width of the tyre and increasing diameters of the interlocking holes to better suit 3D printing.

 ![New wheel assembly](https://github.com/paulcombuse/BadmintonAce/blob/revitalization/documentation/img/Wheel-assembly.png)
 ![Wheel redesign with rim highlighted](https://github.com/paulcombuse/BadmintonAce/blob/revitalization/documentation/img/Wheel-assembly_Rim-highlighted.png)


 Most importantly was playing around with materials, infill and printing patterns. Rectangular patterns create an orthotropic or non-isotropic elasticity - meaning one direction is fairly flexible while another direction feels stiff. We currently use a `Archimedes Chords` pattern for the outer walls and a `Gyroid` infill pattern.

 ![Slicing infill](https://github.com/paulcombuse/BadmintonAce/blob/revitalization/documentation/img/Wheel-assembly_currentSlicing_1.png)
 ![Slicing top layer](https://github.com/paulcombuse/BadmintonAce/blob/revitalization/documentation/img/Wheel-assembly_currentSlicing_2.png)

 Soft TPU means Shore A hardness below 60 and that is tricky to print - though, it is possible with a good printer and an appropriate filament setup. For our validation we used a `Bambulab H2D` printer with TPU 90 (far to stiff) and `Recreus Filaflex™ Foamy` which should go down to a Shore A hardness of 51.
 
 You can by such a filament for instance here (yes, it is quiet expensive):
 https://filamentworld.de/shop/special-filament/filaflex-filament/recreus-filaflex-foamy-tpu-filament-bis-51a-natur-1-75-mm/?switch_shop=b2c

 I am quiet happy with the current result, but I have to point out that surface friction is still not ideal.

## Web interface
The web interface was already very powerfull - almost too powerfull, but with the added simple touch TFT to my mind there is a good alternative. I did only perform minor adaptions by adding the definition of a maximum and minimum servo angle for rotation of the chute tower on the setup page and by introducing custom shot patterns. The last choosen pattern can then be accessed directly by the according button on the touch TFT. I would like to rework the documentation page, but currently this has no priority.

## Localization aka language support
I implemented a language support so in future more languages can be added to the user interface allowing a more comfortable utilization for users in their native language.

## Ideas
* `Manual control` trainer mode, realized by an additional web page which shows a top view of the court and by just touching a spot on the court a shuttle is immediately shooten there keeping the current trajectory setting (or maybe by long pressing or short pressing the `hardness` could be defined)
* Peter wrote me that he was thinking about adding some kind of `remote control` (e.g. with RF remote control switch). 
https://www.amazon.de/Gebildet-Universal-Fernbedienungsschalter-Relais-Empfängermodul-Momentan-Kippriegel-1Empfänger/dp/B0DJX32J75
Maybe we could directly use the existing ESP32 capability for Bluetooth?
https://www.amazon.de/AMATHINGS-Kamera-Auslöser-Fernbedienung-Bluetooth-Technologie-Fernauslöser/dp/B07ZG291YX