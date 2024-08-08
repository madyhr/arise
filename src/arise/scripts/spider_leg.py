#!/usr/bin/env python3
import numpy as np

# credits: remix of commanderguy3001 and rob's tech workbench

class Spider_leg:
    def __init__(self, name, coxa_length, femur_length, tibia_length):
        self.name = name
        self.coxa_length = coxa_length
        self.femur_length = femur_length
        self.tibia_length = tibia_length
        self.angles = np.array([0.,np.deg2rad(35),np.deg2rad(95)])
        self.joints = self.forward_kinematics()

    def get_angles(self):
        return self.angles

    def set_angles(self, angles):
        self.angles = self.clamp_angles(angles)
        return self.get_angles()
    
    def get_target(self):
        # get the target position of the leg tip
        return self.joints[2]

    def clamp_angles(self, angles):
        for idx, ang in enumerate(angles):
            angles[idx] = (ang + np.pi) % (2 * np.pi) - np.pi
        return np.array(angles)

    def inverse_kinematics(self, target = None):
        if target is None:
            target = self.joints[2]

        x, y, z = target

        # these are used if a link is mounted at an angle compared to previous link (such as a curved tibia)
        coxa_angle_offset = 0
        femur_angle_offset = 0
        tibia_angle_offset = 0
        
        if x == 0:
            coxa_angle = np.pi/2
            P = y - self.coxa_length*np.sin(coxa_angle)
        else:    
            coxa_angle = np.arctan(y/x)
            x = x - self.coxa_length*np.cos(coxa_angle)
            P = x / np.cos(coxa_angle)
        if y != 0:
            y = y - self.coxa_length*np.sin(coxa_angle)

        coxa_angle += coxa_angle_offset
        
        H = np.sqrt(P**2 + z**2)

        phi3 = np.arcsin(abs(z)/H)
        phi2 = np.arccos((self.tibia_length**2 + H**2 - self.femur_length**2)/ (2*self.tibia_length*H))
        phi1 = np.arccos((self.femur_length**2 + H**2 - self.tibia_length**2)/(2*self.femur_length*H))

        if z < 0: # z > 0
            femur_angle = phi1 + phi3
        else:
            femur_angle = phi1 - phi3

        tibia_angle = phi1 + phi2

        femur_angle += femur_angle_offset
        tibia_angle += tibia_angle_offset

        angles = np.array([coxa_angle, femur_angle, tibia_angle])           

        self.set_angles(angles)
        self.forward_kinematics()

        return angles

    def forward_kinematics(self, angles = None):
        if angles is None:
            angles = self.angles

        coxa_angle, femur_angle, tibia_angle = angles

        # coxa_offset = 0
        # femur_offset = np.deg2rad(-55)
        # tibia_offset = np.deg2rad(-97)
        # coxa_angle += coxa_offset
        # femur_angle += femur_offset
        # tibia_angle += tibia_offset

        coxa_pos = np.array([self.coxa_length * np.cos(coxa_angle),
                    self.coxa_length * np.sin(coxa_angle),
                    0
                    ])
        
        femur_pos = np.array([self.femur_length * np.cos(coxa_angle) * np.cos(femur_angle),
                     self.femur_length * np.sin(coxa_angle) * np.cos(femur_angle),
                     - self.femur_length * np.sin(femur_angle)
                     ]) # negate sign fo Z composant to get left-handed CS
        femur_pos = np.array([sum(i) for i in zip(femur_pos, coxa_pos)])

        # helper length (distance between coxa-femur joint and tip of leg)
        H = np.sqrt(self.tibia_length ** 2 + self.femur_length **2 - (2 * self.tibia_length * self.femur_length * np.cos(np.pi - tibia_angle)))
        # helper angle (angle in the right angle triangle created by points of coxa-tip-ground)
        phi = np.arccos((self.femur_length**2 + H**2 - self.tibia_length**2)/(2*self.femur_length*H)) - femur_angle

        tibia_pos = np.array([np.cos(coxa_angle) * np.cos(phi) * H,
                     np.sin(coxa_angle) * np.cos(phi) * H,
                     np.sin(phi) * H 
                     ]) # negate sign of Z composant to get left-handed CS
        tibia_pos = np.array([sum(i) for i in zip(tibia_pos, coxa_pos)])

        joints = np.array([coxa_pos, # coxa-femur joint xyz 
                  femur_pos, # femur-tibia joint xyz
                  tibia_pos  # leg (tibia) tip xyz
                  ])
        
        self.joints = joints
        # note: these joint positions are with respect to an origin at the coxa rotation axis
        return joints