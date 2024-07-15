# ARISE hexapod

Hexapod robot hobby project based primarily on MakeYourPet (https://github.com/MakeYourPet/hexapod) with many inspirations from other projects such as Aecert Robotics, James Bruton and of course the MakeYourPet discord. Goal is to make it serve as a platform to learn more about legged robots and how to integrate different sensors to achieve autonomous control using visual odometry / SLAM with ROS2. 

## Mechanical

<img src="https://github.com/madyhr/arise/blob/master/doc/arise_3_legs.gif" width=30% height=30%>

### Previous tasks:
- 3D printing of base frame and leg components
- Assembly of three legs for kinematics testing
- Servo jitter fixed with friction rings made of rubber instead of PLA
- Adjusting coxa, femur, tibia STLs from MakeYourPet to be compatible with M3 screws instead of M2 screws

### Current task: 
Designing lower and upper body frame capable of keeping electronics organized and safe

### Next up: 
As more components arrive, more of the main body will be assembled (all 6 legs soon)

## Software

### Previous tasks:
- Forward and inverse kinematics of leg of known dimensions
- Trajectory computation using Bezier curves and fixed step length
- Adjustable speed mid trajectory
- ROS2 main software structure
- Link between RP2040 on servo2040 board and RPi5
- Established efficient communication protocols between servo controller (servo2040) and RPi5 for velocity commands
- Adjusting inverse kinematics parameters to translate ideal software model to real hardware model

### Current task: 
- Writing up end-of-control movement to reset leg positions after control signals stop

### Next up: 
- Writing the code for using a wireless (PS4) controller to send velocity commands

## Electrical

### Previous tasks:
- Ensuring all components receive nominal voltages and can receive necessary currents
- Preparing servo2040 board for relay module (still waiting for module itself to arrive)

### Current task: 
- Testing relay module and battery power

### Next up: 
- Thinking of solutions to optimize wiring (follows upper/lower frame development)
