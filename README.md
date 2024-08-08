# ARISE hexapod

Hexapod robot hobby project based primarily on MakeYourPet (https://github.com/MakeYourPet/hexapod) with many inspirations from other projects such as Aecert Robotics, James Bruton and of course the MakeYourPet discord. Goal is to make it serve as a platform to learn more about legged robots and how to integrate different sensors to achieve autonomous control using visual odometry / SLAM with ROS2. 

<img src="https://github.com/madyhr/arise/blob/master/doc/arise_3_legs.gif" width=30% height=30%>

## Mechanical

### Previous tasks:
- 3D printing of base frame and leg components
- Assembly of three legs for kinematics testing
- Servo jitter fixed with friction rings made of rubber instead of PLA
- Adjusting coxa, femur, tibia STLs from MakeYourPet to be compatible with M3 screws instead of M2 screws
- All six legs have been fitted to the main body frame

### Current task: 
Designing lower and upper body frame capable of keeping electronics organized and safe

### Next up: 
Designing leg shields for coolness factor so they don't look as feeble and weak.

## Software

### Previous tasks:
- Forward and inverse kinematics of leg of known dimensions
- Trajectory computation using Bezier curves and fixed step length
- Adjustable speed mid trajectory
- ROS2 main software structure
- Link between RP2040 on servo2040 board and RPi5
- Established efficient communication protocols between servo controller (servo2040) and RPi5 for velocity commands
- Adjusting inverse kinematics parameters to translate ideal software model to real hardware model
- Control commands sent through wired controller
- Code organized into ROS2 framework with launch files etc

### Current task: 
- Writing up end-of-control movement to reset leg positions after control signals stop

### Next up: 
- Writing the code for using a wireless instead of wired controller to send velocity commands

## Electrical

### Previous tasks:
- Ensuring all components receive nominal voltages and can receive necessary currents
- Wiring for servo2040 and rover legs through lab power supply tested
- Wiring for battery powered circuit complete but not tested (UBEC, buck converter, switches)

### Current task: 
- Tests of powering everything at once with batteries

### Next up: 
- Thinking of solutions to optimize wiring (follows upper/lower frame development)
