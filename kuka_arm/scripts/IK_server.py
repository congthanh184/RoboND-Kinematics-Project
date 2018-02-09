#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy 

class KukaR210:
    def __init__(self):
        self.alpha0 = self.alpha2 = self.alpha6 = 0
        self.alpha1 = self.alpha3 = self.alpha5 = -pi/2
        self.alpha4 = pi/2

        self.a0 = self.a4 = self.a5 = self.a6 = 0
        self.a1 = 0.35
        self.a2 = 1.25
        self.a3 = -0.054

        self.d2 = self.d3 = self.d5 = self.d6 = 0
        self.d1 = 0.75
        self.d4 = 1.5
        self.dg = 0.303
        self.d34 = 0.96
        self.d45 = 0.54

        qz = tf.transformations.rotation_matrix(pi, (0,0,1))    
        qy = tf.transformations.rotation_matrix(-pi/2, (0,1,0))    
        self.R_corr = numpy.dot(qz, qy)

    def get_dh_transformation(self, alpha, a, d, theta):
        xaxis, zaxis = (1, 0, 0), (0, 0, 1)
        qx = tf.transformations.rotation_matrix(alpha, xaxis)    
        qz = tf.transformations.rotation_matrix(theta, zaxis)    
        ax = tf.transformations.translation_matrix((a, 0, 0))
        dz = tf.transformations.translation_matrix((0, 0, d))
        T = numpy.dot(numpy.dot(qx, ax), numpy.dot(qz, dz))
        return T

    # Get joint2 position 
    def get_joint2_position(self, q1):        
        T0_1 = self.get_dh_transformation(self.alpha0, self.a0, self.d1, q1)
        T1_2 = self.get_dh_transformation(0, self.a1, 0, 0)
        T = numpy.dot(T0_1, T1_2)
        return T[0:3, 3]

    def get_T0_3_inv(self, q1, q2, q3):
        T0_1 = self.get_dh_transformation(self.alpha0, self.a0, self.d1, q1)
        T1_2 = self.get_dh_transformation(self.alpha1, self.a1, self.d2, q2 - (numpy.pi/2))
        T2_3 = self.get_dh_transformation(self.alpha2, self.a2, self.d3, q3)
        T0_3 = numpy.dot(numpy.dot(T0_1, T1_2), T2_3)
        return numpy.linalg.inv(T0_3)

    def get_ee_pose_base(self, position, orientation):
        ee_pose = numpy.dot(tf.transformations.translation_matrix((position.x, position.y, position.z)),
                    tf.transformations.quaternion_matrix((orientation.x, orientation.y, orientation.z, orientation.w)))
        return numpy.dot(ee_pose, self.R_corr)

    def get_wrist_position(self, ee_base):
        return ee_base[0:3, 3] - self.dg * ee_base[0:3, 2]

    def vec_len(self, vec):
        sqr_len = [pos**2 for pos in vec]
        return numpy.sqrt(sum(sqr_len))

    def IK(self, ee_position, ee_orientation):
        # calculate wrist position from ee position and orientation
        ee_base = self.get_ee_pose_base(ee_position, ee_orientation)
        wrist_pos = self.get_wrist_position(ee_base)

        # calculate theta1 by wrist position
        q1 = numpy.arctan2(wrist_pos[1], wrist_pos[0])

        # calculate triangle's side oppsition with theta3
        vec_J2_W = numpy.subtract(wrist_pos, self.get_joint2_position(q1))
        side_B = self.vec_len(vec_J2_W)
        side_d4_cor = numpy.sqrt(self.d4**2 + self.a3**2)
        delta = numpy.arctan2(abs(self.a3), self.d34) - numpy.arctan2(abs(self.a3), self.d4)

        # find theta 3 prime which expresses the relative angle with theta 2
        c3_prime = (side_B**2 - self.a2**2 - side_d4_cor**2) / (2 * self.a2 * side_d4_cor)
        prime3 = numpy.arctan2(numpy.sqrt(1 - (c3_prime**2)), c3_prime)

        # find theta2 and theta3
        beta = numpy.arctan2(vec_J2_W[2], numpy.sqrt(vec_J2_W[0]**2 + vec_J2_W[1]**2))
        gamma = numpy.arctan2(Kuka.d4 * numpy.sin(prime3), Kuka.d4 * numpy.cos(prime3) + Kuka.a2)

        q2 = (numpy.pi/2) - beta - gamma
        q3 = prime3 - (numpy.pi/2) - delta

        # get T3_6 
        T0_3_inv = self.get_T0_3_inv(q1, q2, q3)
        T3_6 = numpy.dot(T0_3_inv, ee_base)

        # calculate theta4, theta5, theta6
        q4 = numpy.arctan2( T3_6[2][2], -T3_6[0][2])
        q5 = numpy.arctan2( numpy.sqrt(1 - T3_6[1][2]**2), T3_6[1][2])
        q6 = numpy.arctan2( -T3_6[1][1], T3_6[1][0])

        return (q1, q2, q3, q4, q5, q6)

Kuka = KukaR210()

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
    	#
    	#
    	# Create Modified DH parameters
    	#
    	#
    	# Define Modified DH Transformation matrix
    	#
    	#
    	# Create individual transformation matrices
    	#
    	#
    	# Extract rotation matrices from the transformation matrices
    	#
    	#
        ###

        joint_trajectory_list = []
        # Initialize service response
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

    	    # Extract end-effector position and orientation from request
    	    # px,py,pz = end-effector position
    	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])


            ### Your IK code here
    	    # Compensate for rotation discrepancy between DH parameters and Gazebo
    	    #
    	    #
    	    # Calculate joint angles using Geometric IK method
    	    #
    	    #
            ###
            position = req.poses[x].position
            orientation = req.poses[x].orientation
            (theta1, theta2, theta3, theta4, theta5, theta6) = Kuka.IK(position, orientation)
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
    	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
    	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
