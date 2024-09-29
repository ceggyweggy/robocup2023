# robocup2023

1st place at RoboCup Singapore Open in 2023, 3rd place at RoboCup International (Bordeaux) in 2023.

The code that is run on the robot is `maincode.ino `, which initialises all the parts of the robot and contains the control flow loop that the robot follows. Our loop time is 14ms. Each component of the robot (i.e. motors, camera, lidar, GY, kicker, bluetooth) has its own library component under `library.h`. `Drivebase` integrates the movement of the four motors on the robot so that the robot can accept an input of speed, angle and rotation and set the motors to the correct speed to achieve the desired movement.
