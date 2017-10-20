#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import numpy as np
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        print("----starting---")
        joint_trajectory_list = []

            # Define DH param symbols

        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # twist angles
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # link lengths
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offsets
        
        
        # Joint angle symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # 'theta' joint angles

  
        # Modified DH params
        #q2 = q2 - pi / 2.
        #q7 = 0

        
        # Define Modified DH Transformation matrix
        s = {alpha0: 0,     a0:   0,    d1: 0.75, 
             alpha1: -pi/2, a1: 0.35,   d2: 0,       q2: q2 - pi/2,  
             alpha2: 0,     a2: 1.25,   d3: 0,
             alpha3: -pi/2, a3: -0.054, d4: 1.5,
             alpha4: pi/2,  a4: 0,      d5: 0,
             alpha5: -pi/2, a5: 0,      d6: 0,
             alpha6: 0,     a6: 0,      d7: 0.303,   q7: 0}



        # Create individual transformation matrices
        T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
           [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
           [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
           [                   0,                   0,            0,               1]])
        
        T0_1 = T0_1.subs(s)

        T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
           [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
           [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
           [                   0,                   0,            0,               1]])
        
        T1_2 = T1_2.subs(s)

        T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
           [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
           [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
           [                   0,                   0,            0,               1]])
        
        T2_3 = T2_3.subs(s)

        T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
           [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
           [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
           [                   0,                   0,            0,               1]])
        
        T3_4 = T3_4.subs(s)

        T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
           [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
           [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
           [                   0,                   0,            0,               1]])
        
        T4_5 = T4_5.subs(s)

        T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
           [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
           [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
           [                   0,                   0,            0,               1]])
        
        T5_6 = T5_6.subs(s)

        T6_7 = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
           [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
           [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
           [                   0,                   0,            0,               1]])
        
        T6_7 = T6_7.subs(s)


        print("starting calculating T0_7")
        # Transform from base link to end effector
        T0_7 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7
        print("starting calculated T0_7")


        #Correction for orientation difference between defintion of gripper link URDF v DH
        
        #first rotate around z-axis by pi
        R_z = Matrix([[             cos(pi),            -sin(pi),            0,              0],
           [                        sin(pi),            cos(pi),             0,              0],
           [                        0,                  0,                   1,              0],
           [                        0,                  0,                   0,              1]])

        #then rotate around y-axis by -pi/2
        R_y = Matrix([[             cos(-pi/2),         0,                   sin(-pi/2),     0],
           [                        0,                  1,                   0,              0],
           [                        -sin(-pi/2),        0,                   cos(-pi/2),     0],
           [                        0,                  0,                   0,              1]])

        #calculate total correction factor
        R_corr = R_z * R_y

        #calculate corrected transform from base to end effector
        #T_total = simplify(T0_7 * R_corr)

        T0_3 = T0_1 * T1_2 * T2_3

        # Extract rotational component of transform matrix
        R0_3 = T0_3[0:3, 0:3]


        r, p, ya = symbols('r p ya')

        R_roll = Matrix([[ 1,              0,        0],
                      [ 0,        cos(r), -sin(r)],
                      [ 0,        sin(r),  cos(r)]])

        R_pitch = Matrix([[ cos(p),        0,  sin(p)],
                      [       0,        1,        0],
                      [-sin(p),        0,  cos(p)]])

        R_yaw = Matrix([[ cos(ya), -sin(ya),        0],
                      [ sin(ya),  cos(ya),        0],
                      [ 0,              0,        1]])

        #R0_6 = simplify(R_roll * R_pitch * R_yaw)
        R0_6 = R_yaw * R_pitch * R_roll
        R0_6 = R0_6 * R_corr[0:3, 0:3]

        print("entering the for loop")
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
     
            # Calculate joint angles using Geometric IK method

            

            ########################## 1) First calculate Wrist Centre (WC) #################################

            ############ 1.a) Construct R0_6 based on target roll, pitch and yaw angles of end-effector ######

            #print("R0_6")
            #print(R0_6.evalf(subs={roll:0, yaw:0, pitch:0}))
            print("calc R0_6_num @ 185")
            R0_6_num = R0_6.evalf(subs={r:roll, ya:yaw, p:pitch})


            #############   1.b) Calculate wrist centre (WC) based on ###################################
            #############   translation along z-axis from EE location ####################################

            EE = Matrix([[px],[py],[pz]])
            #P_EE = Matrix([[2.153],[0],[1.946]])
            #P_EE = Matrix([[-0.18685],[2.1447],[1.9465]])

            WC = EE - (0.303) * R0_6_num[:, 2]
            #print("P_WC")
            #print(P_WC.evalf(subs={roll:0, yaw:0, pitch:0}))

            
            theta1 = atan2( WC[1], WC[0] )

            

            side_a = 1.501
            side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35 ), 2 )  + pow((WC[2] - 0.75), 2 ))
            # print(side_b)
            side_c = 1.25

            angle_a = acos((side_c * side_c + side_b * side_b - side_a * side_a ) / (2 * side_b * side_c))
            angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b ) / (2 * side_a * side_c))
            angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c ) / (2 * side_a * side_b))
            
            theta2 = np.pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
            theta3 = np.pi / 2 - (angle_b + 0.036)
            R0_3_num = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
            print("thetas123 calculated @ 217")
            #rospy.loginfo("R0_3_num", R0_3_num)

            #calculate inverse of R0_3
            #R0_3_num_inv = R0_3_num ** -1
            R0_3_num_inv = R0_3_num.transpose()

            R3_6 = R0_3_num_inv * R0_6_num

            #alpha, beta, gamma = tf.transformations.euler_from_matrix(R3_6, axes = 'ryzx')
            #theta4 = alpha
            #theta5 = beta - pi/2
            #theta6 = gamma - pi/2

            #theta6 = atan2(R3_6[1,0],R3_6[0,0]) # rotation about Z-axis
            #theta5 = atan2(-R3_6[2,0], sqrt(R3_6[0,0]*R3_6[0,0]+R3_6[1,0]*R3_6[1,0])) # rotation about Y-axis
            #theta4 = atan2(R3_6[2,1],R3_6[2,2]) # rotation about X-axis

            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            print("calc thetas456 @ 185")

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
        joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
        joint_trajectory_list.append(joint_trajectory_point)

            #print("Requested poses")
            #print("x", px, "y", py, "z", pz)
            #print("roll", roll, "pitch", pitch, "yaw", yaw)

            #T0_7_num = T0_7.evalf(subs={q1:theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6})
            #print("Forward Kinematics T0_7_num")
            #print(T0_7_num)

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
