# BadmintonAce
This is the repository of CAD & Code for my Hackaday Prize 2023 Entry.
https://hackaday.io/project/190336-badminton-ace-shuttle-launcher

Badminton Ace is a project to create a relatively low cost multi-axis robot capable of launching badminton shuttles in user defined patterns for teaching and training. This project seeks to engage children and adults in the exciting sport of badminton to improve physical health while providing an engaging mechatronics system that demonstrates the power of STEM.

Users or trainers can connect to the launcher using wifi then select and adjust shot placement and pattern to learn or practice new badminton return techniques. This will be done directly in the browser using an interactive 3D court with controls for rotation and trajectory.

The robot can also be used as a powerful STEM educational tool throughout the build process if desired. With multiple sensors, 2 brushless and 4 servo motors, 3D printing, laser cutting, urethane molding, programing, inverse kinematics, and more, this project covers a wide range of topics for those looking to learn how it all works. It also helps to encourage excercise for it's users.


# Badminton Ace Design 4
This section contains the Code for the current design variant 4 for use with an ESP32 microcontroller. The code is all working but I'm not happy with the quality of calibration. The code is also a bit of a mess but it's all here for your enjoyment!

The code is programmed, compiled and uploaded to the ESP32 via the Visual Studio Code IDE with PlatformIO.
The webpage, images, and CSS are all loaded via SPIFFS within VSC IDE with PlatformIO:
    PlatformIO > Platform > Build Filesystem Image + Upload Filesystem Image.

