# ARISE hexapod

Hexapod robot hobby project based primarily on MakeYourPet (https://github.com/MakeYourPet/hexapod) with many inspirations from other projects such as Aecert Robotics, James Brunton and of course the MakeYourPet discord. Goal is to make it serve as a platform to learn more about legged robots and how to integrate different sensors to achieve autonomous control using visual odometry / SLAM with ROS2. 

## Mechanical

![Alt text](/doc/hexapod_leg.jpg?raw=true)

### Previous tasks:
- 3D printing of base frame and leg components
- Assembly of one leg for kinematics testing

### Current task: Attempting to fix servo jitter by use of different screws and friction rings in the 3 joints

### Next up: As more components arrive, more of the main body will be assembled

## Software

### Previous tasks:
- Forward and inverse kinematics of leg of known dimensions
- Trajectory computation using Bezier curves and fixed step length
- Adjustable speed mid trajectory
- ROS2 main software structure
- Link between RP2040 on servo2040 board and RPi5

### Current task: Establishing efficient communication protocols between servo controller (servo2040) and RPi5 for velocity commands

### Next up: Adjusting inverse kinematics parameters to real life conditions to achieve smoother motion

## Electrical

### Previous tasks:
- Ensuring all components receive nominal voltages and can receive necessary currents
- Preparing servo2040 board for relay module (still waiting for module itself to arrive)

### Current task: Thinking of solutions to optimize wiring

### Next up: Implementing more optimal wiring solutions
