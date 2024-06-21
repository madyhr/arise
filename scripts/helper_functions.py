#!/usr/bin/env python3
import math
import numpy as np

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
    # define quadratic bezier curve from 3 points and number of steps
    t = np.linspace(0, 1, steps)[:, None]  # Create a column vector of t values
    one_minus_t = 1 - t
    B_t = P1 + (one_minus_t**2 * (P0 - P1)) + (t**2 * (P2 - P1)) # quadratic bezier curve
    
    return B_t

