#!/usr/bin/env python3
import math
import numpy as np
import serial


def calc_parabola_vertex(x1, y1, x2, y2, x3, y3):
		'''
		Adapted and modifed to get the unknowns for defining a parabola:
		http://stackoverflow.com/questions/717762/how-to-calculate-the-vertex-of-a-parabola-given-three-points
		'''

		denom = (x1-x2) * (x1-x3) * (x2-x3);
		A     = (x3 * (y2-y1) + x2 * (y1-y3) + x1 * (y3-y2)) / denom;
		B     = (x3*x3 * (y1-y2) + x2*x2 * (y3-y1) + x1*x1 * (y2-y3)) / denom;
		C     = (x2 * x3 * (x2-x3) * y1+x3 * x1 * (x3-x1) * y2+x1 * x2 * (x1-x2) * y3) / denom;

		return A,B,C

def rotate_3d(p, origin=(0, 0, 0), degrees=0):
    # rotate point about z-axis passing through origin  
    angle = np.deg2rad(degrees)
    R = np.array([[np.cos(angle), -np.sin(angle), 0],
                  [np.sin(angle),  np.cos(angle), 0],
                  [0, 0, 1]])
    o = np.atleast_2d(origin)
    p = np.atleast_2d(p)
    return np.squeeze((R @ (p.T-o.T) + o.T).T)

def bezier_curve_quad(P0, P1, P2, steps):
    t = np.linspace(0, 1, steps)[:, None]  # Create a column vector of t values
    one_minus_t = 1 - t
    B_t = P1 + (one_minus_t**2 * (P0 - P1)) + (t**2 * (P2 - P1)) # quadratic bezier curve
    
    return B_t

# def orthogonal_projection_midpoint(point1, point2, step_height):
#     # Calculate the midpoint
#     midpoint = (point1 + point2) / 2
    
#     # Calculate the direction vector of the line
#     direction_vector = point2 - point1
    
#     # Define the normal vector to the plane formed by the points and the projection of point1 onto the XY-plane
#     projection_vector = np.array([point1[0], point1[1], 0])
#     normal_vector = np.cross(direction_vector, projection_vector - point1)
    
#     # Find a perpendicular vector in the plane formed by point1, point2, and the projection
#     perpendicular_vector = np.cross(direction_vector, normal_vector)
    
#     # Normalize the perpendicular vector
#     perpendicular_vector = perpendicular_vector / np.linalg.norm(perpendicular_vector)
    
#     # Calculate the new point by moving "step_height" away from the midpoint
#     new_point = midpoint + step_height * perpendicular_vector
    
#     return midpoint, new_point

def create_control_point(P0, P2, scale_factor):
    P0 = np.array(P0)
    P2 = np.array(P2)

    # Midpoint between P0 and P2
    midpoint = (P0 + P2) / 2

    # Distance between P0 and P2
    distance = np.linalg.norm(P2 - P0)

    # Height
    height = - distance * scale_factor

    # Create P1 directly above the midpoint
    P1 = np.array([midpoint[0], midpoint[1], midpoint[2] + height])

    return P1


def calc_velocity(x, y, precision = 2):
    # find speed and direction given joystick x and y
    # specify precision as number of decimals in output

    direction = math.atan2(x, y)

    if x == 0:
        direction = 0 if y > 0 else math.pi
    elif y == 0:
        direction = math.pi / 2 if x > 0 else 3 * math.pi / 2

    direction = direction if direction >= 0 else direction + 2 * math.pi

    speed = min(1.0, math.hypot(x, y))

    return round(speed,precision), round(direction,precision)

def find_port():
    # used for finding the port currently in use by the RP2040
    active_port = "ACM0"

    ports = ["ACM0","ACM1","ACM2"]
    found_port = False
    for usbport in ports:
        if found_port == False:
            try:
                s = serial.Serial(port=f"/dev/tty{usbport}", parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE, timeout=0.005)
                active_port = usbport
                found_port = True
            except serial.SerialException:
                continue

    return active_port