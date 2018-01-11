#!/usr/bin/env python

import numpy as np 
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2
from sympy.matrices import Matrix
import tf

def transform_dh_matrix(s, theta, alpha, d, a):
    """Build the modified DH transformation matrix based on the provided
    theta, alpha, d and a values.
    :param s: Dictionary of DH parameters for the manipulator
    :param theta: Sympy symbol
    :param alpha: Sympy symbol
    :param d: Sympy symbol
    :param a: Sympy symbol
    :return: Sympy Matrix object of the DH transformation matrix
    """
    # Create the transformation matrix template
    Ta_b = Matrix([[            cos(theta),           -sin(theta),           0,             a],
                   [ sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [ sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                     0,                     0,           0,             1]])
    # Substitute in the DH parameters into the matrix
    Ta_b = Ta_b.subs(s)
    return Ta_b

### Define functions for Rotation Matrices about x, y, and z given specific angle.

def rot_x(q):
    R_x = Matrix([  [ 1,        0,       		0	],
                    [ 0,        cos(q), -sin(q)		],
                    [ 0,        sin(q),  cos(q)		]])
    
    return R_x
    
def rot_y(q):              
    R_y = Matrix([[cos(q), 0, sin(q)], [0, 1, 0], [-sin(q), 0, cos(q)]])
    
    return R_y

def rot_z(q):    
    R_z = Matrix([[cos(q), -sin(q), 0],[sin(q), cos(q), 0],[0, 0, 1]])
    
    return R_z
              
# Define DH param symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')    # link offsets
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')    # link lengths

# Modified DH params for KUKA KR210
s = {	alpha0:     0, d1:  0.75, a0:      0,
		alpha1: -pi/2, d2:     0, a1:   0.35, q2: (q2 - pi/2),
		alpha2:     0, d3:     0, a2:   1.25,
		alpha3: -pi/2, d4:  1.50, a3: -0.054,
		alpha4:  pi/2, d5:     0, a4:      0,
		alpha5: -pi/2, d6:     0, a5:      0,
		alpha6:     0, d7: 0.303, a6:      0, q7: 0}

T0_1 = transform_dh_matrix(s, q1, alpha0, d1, a0)
T1_2 = transform_dh_matrix(s, q2, alpha1, d2, a1)
T2_3 = transform_dh_matrix(s, q3, alpha2, d3, a2)
T3_4 = transform_dh_matrix(s, q4, alpha3, d4, a3)
T4_5 = transform_dh_matrix(s, q5, alpha4, d5, a4)
T5_6 = transform_dh_matrix(s, q6, alpha5, d6, a5)
T6_7 = transform_dh_matrix(s, q7, alpha6, d7, a6)

# Correction to account for orientation difference between the gripper and
#   the arm base (rotation around Z axis by 180 deg and Y axis by -90 deg)
R_z = Matrix([[     cos(pi), -sin(pi),          0, 0],
              [     sin(pi),  cos(pi),          0, 0],
              [           0,        0,          1, 0],
              [           0,        0,          0, 1]])
R_y = Matrix([[  cos(-pi/2),        0, sin(-pi/2), 0],
              [           0,        1,          0, 0],
              [ -sin(-pi/2),        0, cos(-pi/2), 0],
              [           0,        0,          0, 1]])

R_corr = R_z * R_y

T_total = (T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7 * R_corr)
mat_transform = T_total.evalf(subs={q1: -0.7, q2: 0.91, q3: -3.38, q4: -1.67, q5: 0.15, q6: 3.47})
print('T_total = ', mat_transform)
mat_np = np.array(mat_transform).astype(np.float64)
print('orientation = ', tf.transformations.quaternion_from_matrix(mat_np))
print(R_corr)
R2 = rot_y(q1) * rot_z(q2)
R2 = R2.evalf(subs={q1: pi/2, q2: pi})
print(R2)
T3_6 = simplify(T3_4 * T4_5 * T5_6)
print('T3_6 = ', T3_6)
