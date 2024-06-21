#!/usr/bin/env python3
from spider_leg import spider_leg
import math
import helper_functions as help

class hexapod:
    '''
    Hexapod overview
    F = forwards, B = backwards, leg numbering convention
       4     5     6  
        \    |    /
         \   |   /
    F    |=======|    B
         /   |   \
        /    |    \
       1     2     3  
    
    '''
    def __init__(self, num_legs,coxa_length, femur_length, tibia_length) -> None:
        self.coxa_length = coxa_length
        self.femur_length = femur_length
        self.tibia_length = tibia_length
        self.legs = self.instantiate_legs(num_legs)
        self.stepvars = [80,50]
        self.gait = "tripod"
        pass

    def instantiate_legs(self, num_legs):
        legs = []
        for i in range(num_legs):
            leg = spider_leg(f"leg{i+1}",
                               coxa_length=self.coxa_length,
                               femur_length=self.femur_length,
                               tibia_length=self.tibia_length)
            legs.append(leg)

        return legs
    
    def initialize_legs(self):
        # send command to initialize legs so that legs are ready to move
        pass
    
    def move_linear(self, direction, speed):
        # send command to move straight in a given direction with a given speed
        # defining direction as an angle between 0 and 2pi, 0 = forwards, pi = backwards
        # defining speed as a float between 0 and 1 that scales step size as needed
        # send command to choose active gait pattern
        step_length, step_height = self.stepvars[0], self.stepvars[1]

        # step_points = 

        match self.gait:

            case "tripod":
                # assuming leg initial position is something like [0, 145, -70]
                # legs are mirrored I guess? so if sequence is 1-3-5, 2-4-6
                # then we can do something like: define 1 (one) new point. then rotate and mirror it to get the other 6
                # should consider doing a "gather step" to do one half cycle on movement start

                

                pass
            
            case "wave":
                pass
        
            case "ripple":
                pass
        pass

    def move_angular(self, direction, speed):
        # send command to rotate in a given direction with a given speed
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

