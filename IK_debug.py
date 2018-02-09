from sympy import *
from time import time
from mpmath import radians
import tf
import numpy

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 

    ## Insert IK code here!
    class KukaDH:
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
        def get_joint2_position(self, theta1):        
            T0_1 = dh_transformation(self.alpha0, self.a0, self.d1, q1)
            T1_2 = dh_transformation(0, self.a1, 0, 0)
            T = numpy.dot(T0_1, T1_2)
            return T[0:3, 3]

        def get_T0_3_inv(self, q1, q2, q3):
            T0_1 = dh_transformation(self.alpha0, self.a0, self.d1, q1)
            T1_2 = dh_transformation(self.alpha1, self.a1, self.d2, q2 - (numpy.pi/2))
            T2_3 = dh_transformation(self.alpha2, self.a2, self.d3, q3)
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
            print('delta', delta)
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


    Kuka = KukaDH()
    kuka_r210 = KukaR210()

    ee_pose = numpy.dot(tf.transformations.translation_matrix((position.x, position.y, position.z)),
                   tf.transformations.quaternion_matrix((orientation.x, orientation.y, orientation.z, orientation.w)))
    qz = tf.transformations.rotation_matrix(pi, (0,0,1))    
    qy = tf.transformations.rotation_matrix(-pi/2, (0,1,0))    
    R_corr = numpy.dot(qz, qy)
    ee_base = numpy.dot(ee_pose, R_corr)
    wrist_pos = ee_base[0:3, 3] - 0.303 * ee_base[0:3, 2]        

    def dh_transformation(alpha, a, d, theta):
        xaxis, zaxis = (1, 0, 0), (0, 0, 1)
        qx = tf.transformations.rotation_matrix(alpha, xaxis)    
        qz = tf.transformations.rotation_matrix(theta, zaxis)    
        ax = tf.transformations.translation_matrix((a, 0, 0))
        dz = tf.transformations.translation_matrix((0, 0, d))
        T = numpy.dot(numpy.dot(qx, ax), numpy.dot(qz, dz))
        return T

    q1 = numpy.arctan2(wrist_pos[1], wrist_pos[0])
    print q1
    def joint2_position(theta1):        
        T0_1 = dh_transformation(Kuka.alpha0, Kuka.a0, Kuka.d1, q1)
        T1_2 = dh_transformation(0, Kuka.a1, 0, 0)
        T = numpy.dot(T0_1, T1_2)
        return T[0:3, 3]

    def vec_len(vec):
        sqr_len = [pos**2 for pos in vec]
        return numpy.sqrt(sum(sqr_len))

    vec_J2_W = numpy.subtract(wrist_pos, joint2_position(q1))
    side_c = vec_len(vec_J2_W)
    c3_prime = (side_c**2 - Kuka.a2**2 - Kuka.d4**2) / (2 * Kuka.a2 * Kuka.d4)

    prime3 = numpy.arctan2(numpy.sqrt(1 - (c3_prime**2)), c3_prime)

    beta = numpy.arctan2(vec_J2_W[2], numpy.sqrt(vec_J2_W[0]**2 + vec_J2_W[1]**2))
    gamma = numpy.arctan2(Kuka.d4 * numpy.sin(prime3), Kuka.d4 * numpy.cos(prime3) + Kuka.a2)

    q2 = (numpy.pi/2) - beta - gamma
    q3 = prime3 - (numpy.pi/2)

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    # print roll, pitch, yaw
    # xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)
    # Rrpy = numpy.dot(   numpy.dot(tf.transformations.rotation_matrix(yaw, zaxis), tf.transformations.rotation_matrix(pitch, yaxis)), 
    #                     numpy.dot(tf.transformations.rotation_matrix(roll, xaxis), R_corr))
    # print '********************'
    # print Rrpy
    # print ee_base
    # print tf.transformations.euler_from_matrix(Rrpy)
    # T0_1 = dh_transformation(Kuka.alpha0, Kuka.a0, Kuka.d1, q1)
    # T1_2 = dh_transformation(Kuka.alpha1, Kuka.a1, Kuka.d2, q2 - (numpy.pi/2))
    # T2_3 = dh_transformation(Kuka.alpha2, Kuka.a2, Kuka.d3, q3)
    # T0_3 = numpy.dot(numpy.dot(T0_1, T1_2), T2_3)

    # print T0_3
    # T3_6 = numpy.dot(numpy.linalg.inv(T0_3), Rrpy)
    # print T3_6
    # print T3_6[1][2], numpy.sqrt(1-(T3_6[1][2]**2)), numpy.arctan2( numpy.sqrt(1 - T3_6[1][2]**2), T3_6[1][2])

    # T1_2 = dh_transformation()
    # T2_3 = dh_transformation()
    # T3_4 = dh_transformation()
    # T4_5 = dh_transformation()
    # T5_6 = dh_transformation()
    # T6_7 = dh_transformation()       

    # T0_2 = 
    # theta1 = numpy.arctan2(wrist_pos[1], wrist_pos[0])
    # theta2 = (numpy.pi/2) - beta - gamma
    # theta3 = prime3 - (numpy.pi/2)
    # theta4 = numpy.arctan2( T3_6[2][2], -T3_6[0][2])
    # theta5 = numpy.arctan2( numpy.sqrt(1 - T3_6[1][2]**2), T3_6[1][2])
    # theta6 = numpy.arctan2( -T3_6[1][1], T3_6[1][0])

    (theta1, theta2, theta3, theta4, theta5, theta6) = kuka_r210.IK(position, orientation)


    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    Test = [None] * 8
    Test[0] = dh_transformation(Kuka.alpha0, Kuka.a0, Kuka.d1, theta1)
    Test[1] = dh_transformation(Kuka.alpha1, Kuka.a1, Kuka.d2, theta2 - (numpy.pi/2))
    Test[2] = dh_transformation(Kuka.alpha2, Kuka.a2, Kuka.d3, theta3)
    Test[3] = dh_transformation(Kuka.alpha3, Kuka.a3, Kuka.d4, theta4)
    Test[4] = dh_transformation(Kuka.alpha4, Kuka.a4, Kuka.d5, theta5)
    Test[5] = dh_transformation(Kuka.alpha5, Kuka.a5, Kuka.d6, theta6)
    Test[6] = dh_transformation(Kuka.alpha6, Kuka.a6, Kuka.dg, 0)
    Test[7] = R_corr

    Test0_g = tf.transformations.identity_matrix()
    Test0_w = tf.transformations.identity_matrix()
    count = 0
    for tm in Test:
        count += 1
        Test0_g = numpy.dot(Test0_g, tm)
        if count == 6:
            Test0_w = numpy.dot(Test0_g, R_corr)
    print Test0_g[0:3, 3], Test0_w[0:3, 3]
    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = Test0_w[0:3, 3] # <--- Load your calculated WC values in this array
    your_ee = Test0_g[0:3, 3] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 3

    test_code(test_cases[test_case_number])
