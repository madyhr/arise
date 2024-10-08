#!/usr/bin/env python3
from spider_leg import Spider_leg
import math
from helper_functions import bezier_curve_quad, rotate_3d, create_control_point
import numpy as np

class Hexapod:
    """
    Hexapod overview
    F = forwards, B = backwards, leg numbering convention
       4     5     6  
        L    |    /
         L   |   /
    B    |=======|    F
         /   |   L
        /    |    L
       1     2     3  
    
    """
    def __init__(self, num_legs,coxa_length, femur_length, tibia_length) -> None:
        self.coxa_length = coxa_length
        self.femur_length = femur_length
        self.tibia_length = tibia_length
        self.legs = self.instantiate_legs(num_legs)
        self.offset_angles = [0, ]
        self.gait = "tripod"
        self.angle_list = [
            [0,0,0],[0,0,0],[0,0,0],
            [0,0,0],[0,0,0],[0,0,0]
        ]
        self.current_direction = 0
        self.current_step_length = 70
        self.t = 0 # timekeeping 
        self.t_transfer = 0
        self.transfer_traj = False
        self.leg_state = "lift"
        self.current_leg_pos = [None,None]
        self.L1 = None
        self.target_pos2 = None
        pass
        

    def instantiate_legs(self, num_legs):
        legs = []
        for i in range(num_legs):
            leg = Spider_leg(f"leg{i+1}",
                               coxa_length=self.coxa_length,
                               femur_length=self.femur_length,
                               tibia_length=self.tibia_length)
            legs.append(leg)

        return legs
    
    def initialize_legs(self):
        # send command to initialize legs so that legs are ready to move

        pass
    
    def move_linear(self, speed, direction):
        # send command to move straight in a given direction with a given speed
        # defining direction as an angle between 0 and 2pi, 0 = forwards, pi = backwards
        default_x, default_y, default_z = 120, 0, 70
        origin_offset = np.array([default_x, default_y, default_z])

        match self.gait:

            case "tripod":

                # gait specific parameters
                max_step_length = 85
                step_height = 70
                t_step = 0.02
                t_tolerance = 0.01 # time tolerance around which points in time a new velocity can be given
                t1_offset = 9/12 # 6 increments for lift phase, 6 increments for push phase, we can use this to set different offsets for each leg to create gaits :)) - Aecert Robotics' idea
                t2_offset = 3/12
                push_fraction = 3/6 # how much time is spent pushing
                num_steps = 100 # may need tuning if not smooth enough
                if speed == 0:  
                    pass # TODO: initialize the end of movement sequence (adjusting legs to be in default positions while maintaining body position)

                else:
                    step_length = speed * max_step_length
                    # only change direction when at "zenith"
                    # TODO: fix why it can receive many direction changes if you move the joysticks quickly
                    if (abs((self.t % 1) - 0) < t_tolerance) or ((abs(self.t) - 1) < (t_tolerance)):
                        self.current_direction = direction
                        self.current_step_length = step_length
                    if abs((self.t % 1) - 0.50) < t_tolerance:
                        self.current_direction = direction
                        self.current_step_length = step_length

                    if self.current_direction < 0:
                        self.current_direction = self.current_direction + 2*np.pi
                    P0 = np.array([np.sin(self.current_direction) * -self.current_step_length/2, 
                                np.cos(self.current_direction) * -self.current_step_length/2, 
                                0]) # start of lift
                    P1 = np.array([0, 0, -step_height]) # control
                    P2 = np.array([np.sin(self.current_direction) * self.current_step_length/2, 
                                np.cos(self.current_direction) * self.current_step_length/2, 
                                0]) # end of lift
                    
                    lift_traj = bezier_curve_quad(P0,P1,P2,int(np.ceil(num_steps*(1-push_fraction))))
                    # lift_traj = np.linspace(P0,P2,int(np.ceil(steps*(1-push_fraction))))
                    push_traj = np.linspace(P2,P0,int(np.floor(num_steps*(push_fraction))))
                    trajectory = np.concatenate([lift_traj,push_traj])

                    t1 = (self.t + t1_offset) % 1
                    t2 = (self.t + t2_offset) % 1

                    traj_index1 = int(np.ceil((len(trajectory)-1)*t1))
                    traj_index2 = int(np.ceil((len(trajectory)-1)*t2))

                    target_pos1 = trajectory[traj_index1]
                    target_pos2 = trajectory[traj_index2]


                    # leg 1,3,5 follow target_pos1 (starts in push phase at y = 0)
                    # leg 2,4,6 follow target_pos2 (starts in lift phase at y = 0)

                    theta = 45 # angular offset of legs in degrees, should maybe be radians instead (also change rotate() then)
                    ## WARNING: remember that Z axis points towards the ground. This means CW and CCW are in reference to this coordinate system (i.e. reversed as seen from the top)
                    target_pos1_mir = np.array([-target_pos1[0], -target_pos1[1], target_pos1[2]])
                    target_pos2_mir = np.array([-target_pos2[0], -target_pos2[1], target_pos2[2]])
                    target_pos2_mir_cw = rotate_3d(target_pos2_mir, degrees = -theta)
                    target_pos2_mir_ccw = rotate_3d(target_pos2_mir, degrees = theta)
                    target_pos1_cw = rotate_3d(target_pos1, degrees = -theta)
                    target_pos1_ccw = rotate_3d(target_pos1, degrees = theta)
                    
                    
                    leg1_angles = self.legs[0].inverse_kinematics(target = np.add(target_pos1_cw,origin_offset)) # leg 1, rotate theta deg CCW
                    leg2_angles = self.legs[1].inverse_kinematics(target = np.add(target_pos2,origin_offset)) # leg 2, baseline 
                    leg3_angles = self.legs[2].inverse_kinematics(target = np.add(target_pos1_ccw,origin_offset)) # leg 3, rotate theta deg CW
                    leg4_angles = self.legs[3].inverse_kinematics(target = np.add(target_pos2_mir_ccw,origin_offset)) # leg 4, mirror x, rotate theta deg CW
                    leg5_angles = self.legs[4].inverse_kinematics(target = np.add(target_pos1_mir,origin_offset)) # leg 5, mirror x
                    leg6_angles = self.legs[5].inverse_kinematics(target = np.add(target_pos2_mir_cw,origin_offset)) # leg 6, mirror x, rotate theta deg CCW
                    
                    # prepping leg angles to be sent via serial to RP2040
                    leg1_msg = [float(np.rad2deg(x)) for x in list(leg1_angles)]
                    leg2_msg = [float(np.rad2deg(x)) for x in list(leg2_angles)]
                    leg3_msg = [float(np.rad2deg(x)) for x in list(leg3_angles)]
                    leg4_msg = [float(np.rad2deg(x)) for x in list(leg4_angles)]
                    leg5_msg = [float(np.rad2deg(x)) for x in list(leg5_angles)]
                    leg6_msg = [float(np.rad2deg(x)) for x in list(leg6_angles)]
                    
                    # angle_list = [leg1_msg, leg2_msg, leg3_msg, leg4_msg, leg5_msg, leg6_msg] # the correct format (before assembly)
                    angle_list = [leg6_msg, leg3_msg, leg5_msg, leg2_msg, leg4_msg, leg1_msg] # the correct format (after assembly)
                    # angle_list = [leg6_msg, leg5_msg, leg4_msg, leg3_msg, leg2_msg, leg1_msg]
                    
                    femur_ang_offset = 35 # angular offset in degrees from assembly
                    tibia_ang_offset = 90 # angular offset in degrees from assembly, 95 org. val

                    # Translating between software model and physical model with different rotation directions and offsets
                    for i in range(len(angle_list)):
                        angle_list[i][1] = -angle_list[i][1] + femur_ang_offset
                        angle_list[i][2] = -angle_list[i][2] + tibia_ang_offset
                    
                    self.angle_list = angle_list
                    self.t += t_step

                pass
            
            case "wave":
                pass
        
            case "ripple":
                pass
        pass
    
    def move_angular(self, direction, speed):
        # send command to rotate in a given direction with a given speed

        default_x, default_y, default_z = 120, 0, 70
        origin_offset = np.array([default_x, default_y, default_z])
        max_step_length = 85
        step_height = 70
        t_step = 0.01
        t_tolerance = 0.01 # time tolerance around which points in time a new velocity can be given
        t1_offset = 9/12 # 6 increments for lift phase, 6 increments for push phase, we can use this to set different offsets for each leg to create gaits :)) - Aecert Robotics' idea
        t2_offset = 3/12
        push_fraction = 3/6 # how much time is spent pushing
        num_steps = 100 # may need tuning if not smooth enough
        if speed == 0:  
            pass # TODO: initialize the end of movement sequence (adjusting legs to be in default positions while maintaining body position)

        else:
            step_length = speed * max_step_length
            # only change direction when at "zenith"
            # TODO: fix why it can receive many direction changes if you move the joysticks quickly
            if (abs((self.t % 1) - 0) < t_tolerance) or ((abs(self.t) - 1) < (t_tolerance)):
                # self.current_direction = direction
                self.current_step_length = step_length
            if abs((self.t % 1) - 0.50) < t_tolerance:
                # self.current_direction = direction
                self.current_step_length = step_length

            if self.current_direction < 0:
                self.current_direction = self.current_direction + 2*np.pi
            P0 = np.array([0, 
                        -self.current_step_length/2, 
                        0]) # start of lift
            P1 = np.array([0, 0, -step_height]) # control
            P2 = np.array([0, 
                        self.current_step_length/2, 
                        0]) # end of lift
            
            lift_traj = bezier_curve_quad(P0,P1,P2,int(np.ceil(num_steps*(1-push_fraction))))
            # lift_traj = np.linspace(P0,P2,int(np.ceil(steps*(1-push_fraction))))
            push_traj = np.linspace(P2,P0,int(np.floor(num_steps*(push_fraction))))
            trajectory = np.concatenate([lift_traj,push_traj])

            t1 = (self.t + t1_offset) % 1
            t2 = (self.t + t2_offset) % 1

            traj_index1 = int(np.ceil((len(trajectory)-1)*t1))
            traj_index2 = int(np.ceil((len(trajectory)-1)*t2))

            target_pos1 = trajectory[traj_index1]
            target_pos2 = trajectory[traj_index2]

            # leg 1,3,5 follow target_pos1 (starts in push phase at y = 0)
            # leg 2,4,6 follow target_pos2 (starts in lift phase at y = 0)
        
            leg1_angles = self.legs[0].inverse_kinematics(target = np.add(target_pos1,origin_offset)) # leg 1, rotate theta deg CCW
            leg2_angles = self.legs[1].inverse_kinematics(target = np.add(target_pos2,origin_offset)) # leg 2, baseline 
            leg3_angles = self.legs[2].inverse_kinematics(target = np.add(target_pos1,origin_offset)) # leg 3, rotate theta deg CW
            leg4_angles = self.legs[3].inverse_kinematics(target = np.add(target_pos2,origin_offset)) # leg 4, mirror x, rotate theta deg CW
            leg5_angles = self.legs[4].inverse_kinematics(target = np.add(target_pos1,origin_offset)) # leg 5, mirror x
            leg6_angles = self.legs[5].inverse_kinematics(target = np.add(target_pos2,origin_offset)) # leg 6, mirror x, rotate theta deg CCW
            
            # prepping leg angles to be sent via serial to RP2040
            leg1_msg = [float(np.rad2deg(x)) for x in list(leg1_angles)]
            leg2_msg = [float(np.rad2deg(x)) for x in list(leg2_angles)]
            leg3_msg = [float(np.rad2deg(x)) for x in list(leg3_angles)]
            leg4_msg = [float(np.rad2deg(x)) for x in list(leg4_angles)]
            leg5_msg = [float(np.rad2deg(x)) for x in list(leg5_angles)]
            leg6_msg = [float(np.rad2deg(x)) for x in list(leg6_angles)]
            
            # angle_list = [leg1_msg, leg2_msg, leg3_msg, leg4_msg, leg5_msg, leg6_msg] # the correct format (before assembly)
            angle_list = [leg6_msg, leg3_msg, leg5_msg, leg2_msg, leg4_msg, leg1_msg] # the correct format (after assembly)
            # angle_list = [leg6_msg, leg5_msg, leg4_msg, leg3_msg, leg2_msg, leg1_msg]
            
            femur_ang_offset = 35 # angular offset in degrees from assembly
            tibia_ang_offset = 90 # angular offset in degrees from assembly, 95 org. val

            # Translating between software model and physical model with different rotation directions and offsets
            for i in range(len(angle_list)):
                angle_list[i][1] = -angle_list[i][1] + femur_ang_offset
                angle_list[i][2] = -angle_list[i][2] + tibia_ang_offset
            
            self.angle_list = angle_list
            self.t += t_step

        pass

    def move_end(self):
        # send command to adjust legs to default position after command is done
        pass

    def set_gait(self, gait):
        # set gait. choose between
        # "tripod", (in dev)
        # "wave", (TODO)
        # "ripple" (TODO)
        self.gait = gait
        return gait
    
    def set_step_vars(self, stepvars):
        # stepvars is here a list of length 2: [step_length, step_height]
        self.stepvars = stepvars 

        return stepvars


# def move_linear(self, speed, direction):
#         # send command to move straight in a given direction with a given speed
#         # defining direction as an angle between 0 and 2pi, 0 = forwards, pi = backwards
#         # send command to choose active gait pattern
#         # step_length, step_height = self.stepvars[0], self.stepvars[1]
#         default_x, default_y, default_z = 140, 0, 70
#         origin_offset = np.array([default_x, default_y, default_z])
#         # get current position
#         # determine end point for both lift and push using direction
#         # find trajectory for lift and push between current and end points
#         # at every call determine the entire trajectories for one whole step length for both lift and push phase legs
#         # send just next position in these trajectories
#         # 

#         match self.gait:

#             case "tripod":

#                 # gait specific parameters
#                 step_length = 70
#                 step_height = 70
#                 t_step = 0.01
#                 t1_offset = 9/12 # 6 increments for lift phase, 6 increments for push phase, we can use this to set different offsets for each leg to create gaits :)) - Aecert Robotics' idea
#                 t2_offset = 3/12
#                 push_fraction = 3/6 # how much time is spent pushing
#                 max_speed = 50
#                 target_speed = max_speed * speed
#                 num_steps = 100 # may need tuning if not smooth enough
#                 new_direction = direction
#                 if target_speed == 0:  
#                     pass # initialize the end of movement sequence (adjusting legs to be in default positions while maintaining body position)

#                 else:
                    
#                     if new_direction != self.prev_direction:
#                         # initiate transfer trajectory
#                         leg2_current_pos = self.legs[1].joints[2] - origin_offset
#                         leg5_current_pos = self.legs[4].joints[2] - origin_offset
#                         self.current_leg_pos = [leg2_current_pos, leg5_current_pos]
                        
#                         self.transfer_traj = True
                    

#                     if self.transfer_traj == True:

#                         if direction < 0:
#                             direction = direction + 2*np.pi

#                         if (self.t % 1) < 0.50: # leg 2 in lift phase
#                             self.leg_state = "lift"
                            
#                         else: # leg 2 in push phase
#                             self.leg_state = "push"
                            
 
#                         if self.leg_state == "push":
#                             # L0 = leg5_current_pos
#                             L0 = self.current_leg_pos[1]
#                             L2 = np.array([
#                                 np.sin(direction) * step_length/2, 
#                                 np.cos(direction) * step_length/2, 
#                                 0
#                             ]) # end of lift
#                             L1 = create_control_point(L0, L2, 0.5)
#                             self.L1 = L2
#                             # P0 = leg2_current_pos
#                             P0 = self.current_leg_pos[0]
#                             P2 = np.array([
#                                 np.sin(direction) * -step_length/2, 
#                                 np.cos(direction) * -step_length/2, 
#                                 0
#                             ]) # start of lift    
                            
#                             lift_traj = bezier_curve_quad(L0,L1,L2, steps = num_steps)
#                             push_traj = np.linspace(P0,P2, num = num_steps)

#                             trajectory1 = lift_traj
#                             trajectory2 = push_traj
                    
#                         elif self.leg_state == "lift":
#                             # L0 = leg2_current_pos
#                             L0 = self.current_leg_pos[0]
#                             L2 = np.array([
#                                 np.sin(direction) * step_length/2, 
#                                 np.cos(direction) * step_length/2, 
#                                 0
#                             ]) # end of lift
#                             L1 = create_control_point(L0, L2, 0.5)
#                             self.L1 = L2
#                             # P0 = leg5_current_pos
#                             P0 = self.current_leg_pos[1]
#                             P2 = np.array([
#                                 np.sin(direction) * -step_length/2, 
#                                 np.cos(direction) * -step_length/2, 
#                                 0
#                             ]) # start of lift    
                            
#                             lift_traj = bezier_curve_quad(L0,L1,L2, steps = num_steps)
#                             push_traj = np.linspace(P0,P2, num = num_steps)

#                             trajectory1 = push_traj
#                             trajectory2 = lift_traj

#                         traj_index = int(np.ceil((len(trajectory1)-1)*self.t_transfer))*2

#                         target_pos1 = trajectory1[traj_index]
#                         target_pos2 = trajectory2[traj_index]

#                         self.target_pos2 = target_pos2
                        
#                         self.t_transfer += t_step 
                        
                        
#                         # if transfer trajectory is over
#                         if (self.t_transfer) >= 0.50:
#                             if self.leg_state == "lift":
#                                 self.t = 0.5
#                             elif self.leg_state == "push":
#                                 self.t = 0

#                             self.t_transfer = 0
#                             self.transfer_traj = False

                                       

#                     else: # normal trajectory 
#                         # here the normal trajectory should be calculated
#                         if direction < 0:
#                             direction = direction + 2*np.pi
#                         P0 = np.array([np.sin(direction) * -step_length/2, 
#                                     np.cos(direction) * -step_length/2, 
#                                     0]) # start of lift
#                         P1 = np.array([0, 0, -step_height]) # control
#                         P2 = np.array([np.sin(direction) * step_length/2, 
#                                     np.cos(direction) * step_length/2, 
#                                     0]) # end of lift
                        
#                         lift_traj = bezier_curve_quad(P0,P1,P2,int(np.ceil(num_steps*(1-push_fraction))))
#                         # lift_traj = np.linspace(P0,P2,int(np.ceil(steps*(1-push_fraction))))
#                         push_traj = np.linspace(P2,P0,int(np.floor(num_steps*(push_fraction))))
#                         trajectory = np.concatenate([lift_traj,push_traj])

#                         t1 = (self.t + t1_offset) % 1
#                         t2 = (self.t + t2_offset) % 1

#                         traj_index1 = int(np.ceil((len(trajectory)-1)*t1))
#                         traj_index2 = int(np.ceil((len(trajectory)-1)*t2))

#                         target_pos1 = trajectory[traj_index1]
#                         target_pos2 = trajectory[traj_index2] 

#                         self.target_pos2 = target_pos2

#                         self.t += t_step


#                     # leg 1,3,5 follow target_pos1 (starts in push phase at y = 0)
#                     # leg 2,4,6 follow target_pos2 (starts in lift phase at y = 0)

#                     theta = 45 # angular offset of legs in degrees, should maybe be radians instead (also change rotate() then)
#                     ## WARNING: remember that Z axis points towards the ground. This means CW and CCW are in reference to this coordinate system (i.e. reversed as seen from the top)
#                     target_pos1_mir = np.array([-target_pos1[0], -target_pos1[1], target_pos1[2]])
#                     target_pos2_mir = np.array([-target_pos2[0], -target_pos2[1], target_pos2[2]])
#                     target_pos2_mir_cw = rotate_3d(target_pos2_mir, degrees = -theta)
#                     target_pos2_mir_ccw = rotate_3d(target_pos2_mir, degrees = theta)
#                     target_pos1_cw = rotate_3d(target_pos1, degrees = -theta)
#                     target_pos1_ccw = rotate_3d(target_pos1, degrees = theta)
                    
                    
#                     leg1_angles = self.legs[0].inverse_kinematics(target = np.add(target_pos1_cw,origin_offset)) # leg 1, rotate theta deg CCW
#                     leg2_angles = self.legs[1].inverse_kinematics(target = np.add(target_pos2,origin_offset)) # leg 2, baseline 
#                     leg3_angles = self.legs[2].inverse_kinematics(target = np.add(target_pos1_ccw,origin_offset)) # leg 3, rotate theta deg CW
#                     leg4_angles = self.legs[3].inverse_kinematics(target = np.add(target_pos2_mir_ccw,origin_offset)) # leg 4, mirror x, rotate theta deg CW
#                     leg5_angles = self.legs[4].inverse_kinematics(target = np.add(target_pos1_mir,origin_offset)) # leg 5, mirror x
#                     leg6_angles = self.legs[5].inverse_kinematics(target = np.add(target_pos2_mir_cw,origin_offset)) # leg 6, mirror x, rotate theta deg CCW
                    
#                     # prepping leg angles to be sent via serial to RP2040
#                     leg1_msg = [float(np.rad2deg(x)) for x in list(leg1_angles)]
#                     leg2_msg = [float(np.rad2deg(x)) for x in list(leg2_angles)]
#                     leg3_msg = [float(np.rad2deg(x)) for x in list(leg3_angles)]
#                     leg4_msg = [float(np.rad2deg(x)) for x in list(leg4_angles)]
#                     leg5_msg = [float(np.rad2deg(x)) for x in list(leg5_angles)]
#                     leg6_msg = [float(np.rad2deg(x)) for x in list(leg6_angles)]
                    
#                     angle_list = [leg1_msg, leg2_msg, leg3_msg, leg4_msg, leg5_msg, leg6_msg] # the correct format
#                     # angle_list = [leg6_msg, leg5_msg, leg4_msg, leg3_msg, leg2_msg, leg1_msg]
                    
#                     femur_ang_offset = 35 # angular offset in degrees from assembly
#                     tibia_ang_offset = 95 # angular offset in degrees from assembly

#                     # Translating between software model and physical model with different rotation directions and offsets
#                     for i in range(len(angle_list)):
#                         angle_list[i][1] = -angle_list[i][1] + femur_ang_offset
#                         angle_list[i][2] = -angle_list[i][2] + tibia_ang_offset
                    
#                     self.angle_list = angle_list
#                     self.prev_direction = new_direction

#                 pass
            
#             case "wave":
#                 pass
        
#             case "ripple":
#                 pass
#         pass