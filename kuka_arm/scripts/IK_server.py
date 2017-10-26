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
import tf
import numpy as np
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
        
        # defing the symbols for use
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6            = symbols('alpha0:7')
        a0, a1, a2, a3, a4, a5, a6                                        = symbols('a0:7')
        d1, d2, d3, d4, d5, d6, d7                                        = symbols('d1:8')
        q1, q2, q3, q4, q5, q6, q7                                        = symbols('q1:8')

        r, p, y                                                           = symbols('r p y')
        a, d, q, alpha                                                    = symbols('a d q alpha')

        x_angle, y_angle, z_angle                                         = symbols('x_angle y_angle z_angle')
        x_d, y_d, z_d                                                     = symbols('x_d y_d z_d')

        # known DH parameters 
        s = {
                        alpha0: 0       , a0: 0     , d1: 0.75,    
                        alpha1: -pi/2.   , a1: 0.35  , d2: 0. ,  q2: q2 - pi/2.,  
                        alpha2: 0.       , a2: 1.25  , d3: 0. ,    
                        alpha3: -pi/2.   , a3: -0.054, d4: 1.5,    
                        alpha4: pi/2.    , a4: 0.     , d5: 0.,    
                        alpha5: -pi/2.   , a5: 0.     , d6: 0.,    
                        alpha6: 0.       , a6: 0.     , d7: 0.303 , q7: 0.   
                }

        # Homogeneous Transformation matrices for elementary operations about the 3 orthogonal axes

        Tx = Matrix([[      1,            0,                  0  ,    x_d  ],
                      [     0,       cos(x_angle),  -sin(x_angle),    0    ],
                      [     0,       sin(x_angle),   cos(x_angle),    0    ],
                      [     0,            0,                    0,    1    ]                 
            ])

        Ty = Matrix([[ cos(y_angle)  ,        0,     sin(y_angle),     0    ],
                      [             0,        1,                0,     y_d  ],
                      [ -sin(y_angle),        0,     cos(y_angle),     0    ],
                      [             0,        0,                0,     1    ]
            ])

        Tz = Matrix([[ cos(z_angle) ,        -sin(z_angle) ,     0,     0   ],
                      [ sin(z_angle),         cos(z_angle) ,     0,     0   ],
                      [            0,                    0 ,     1,     z_d ],
                      [            0,                    0 ,     0,     1   ]
            ])

        # Transformation from frame (i-1) to (i)

        # Using the modified DH parameters, the transformation matrix from frame (i-1) to i,
        # given by the 4 elementary operations

        # Rotation of alpha(i - 1) about the X axis.
        # - Displacement of a(i - 1) along the X axis.
        # (mathematically represented by Tx)
        # - Rotation of theta(i) about the Z axis.
        # - Displacement of a(i) along the Z axis.
        # (mathematically represented by Tz)

        # THerefore the transform is given by Tx * Tz

        Ti_minus_1_i = Tx.subs({x_angle: alpha, x_d: a}) * Tz.subs({z_angle: q, z_d: a})


        # Ti_minus_1_i = Matrix([[cos(q),            -sin(q),            0,              a],
        #             [ sin(q)*cos(alpha), cos(q)*cos(alpha),  -sin(alpha), -sin(alpha)*d],
        #             [ sin(q)*sin(alpha), cos(q)*sin(alpha),   cos(alpha),  cos(alpha)*d],
        #             [                 0,                 0,            0,              1]])

        T0_1 = Ti_minus_1_i.subs({q: q1, alpha: alpha0, a: a0, d: d1})

        # substituting in the values from the known DH parameter table
        T0_1 = T0_1.subs(s)

        T1_2 = Ti_minus_1_i.subs({q: q2, alpha: alpha1, a: a1, d: d2})
        T1_2 = T1_2.subs(s)

        T2_3 = Ti_minus_1_i.subs({q: q3, alpha: alpha2, a: a2, d: d3})
        T2_3 = T2_3.subs(s)

        T3_4 = Ti_minus_1_i.subs({q: q4, alpha: alpha3, a: a3, d: d4})
        T3_4 = T3_4.subs(s)

        T4_5 =  Ti_minus_1_i.subs({q: q5, alpha: alpha4, a: a4, d: d5})
        T4_5 = T4_5.subs(s)

        T5_6 = Ti_minus_1_i.subs({q: q6, alpha: alpha5, a: a5, d: d6}) 
        T5_6 = T5_6.subs(s)

        T6_G = Ti_minus_1_i.subs({q: q7, alpha: alpha6, a: a6, d: d7})
        T6_G = T6_G.subs(s)

        # Homogenous transformation matrxi from base_frame to the gripper

        T0_3 = T0_1 * T1_2 * T2_3
        T0_G = T0_3 * T3_4 * T4_5 * T5_6 * T6_G 

        ## Correcting the orientaion of the gripper

        # Rotation pi radians about Z - axis
        R_z = Tz.subs({z_angle: pi, z_d: 0})
        
        # Rotation -pi/2 radians about Y - axis
        R_y = Ty.subs({y_angle: -pi/2, y_d: 0})
        
        # Transformation matrix for correcting the gripper orientation wrt WC
        R_corr = R_z * R_y

        # Extracting the rotation matrix for 0 to 3 frame 
        # Its inverse will be used to find R3_6
        R0_3 = T0_3[0:3, 0:3]

        # Rotation matrix corresponding to roll, pitch and yaw
        Rx = Tx.subs({x_angle: r})[0:3, 0:3]
        Ry = Ty.subs({y_angle: p})[0:3, 0:3]
        Rz = Tz.subs({z_angle: y})[0:3, 0:3]

        # Rotation matrix to transform to given roll, pitch and yaw with the  correct gripper orientation
        
        R_EE = Rz * Ry * Rx

        R_EE = R_EE * R_corr[0:3, 0:3]

        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            joint_trajectory_point = JointTrajectoryPoint()

            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            # Numerically substituted R_EE with given rpy
            R_EE_num = R_EE.evalf(subs={ r: roll, p: pitch, y: yaw })

            # EE coordinates
            EE = Matrix([[px],[py],[pz]])

            # Wrist center coordinates
            WC = EE - (0.303) * R_EE_num[:, 2]
            Xc = WC[0]
            Yc = WC[1]
            Zc = WC[2]

            # theta1 = atan2( y, x )
            theta1 = atan2( Yc, Xc )
            
            # math.sqrt(0.054 ** 2 + 1.5 ** 2)   ===   1.500971685275908
            side_a = 1.501

            # WC_X_wrt_j1 = sqrt(x ** 2 + y ** 2) - 0.35
            # WC_Z_wrt_j1 = z - 0.75

            # side_b = sqrt(pow(( sqrt(x ** 2 + y ** 2) - 0.35 ), 2 )  + pow((z - 0.75), 2 ))
            side_b = sqrt(pow((sqrt( Xc ** 2 + Yc ** 2 ) - 0.35 ), 2 )  + pow(( Zc - 0.75), 2 ))
            
            # joint3(Z) - joint2(Z)
            side_c = 1.25

            # By cosine rule
            angle_a = acos((side_c * side_c + side_b * side_b - side_a * side_a ) / (2 * side_b * side_c))
            angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b ) / (2 * side_a * side_c))
            angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c ) / (2 * side_a * side_b))
            
            theta2 = np.pi / 2 - angle_a - atan2( Zc - 0.75, sqrt( Xc ** 2 + Yc ** 2 ) - 0.35)

            # TODO: make all angles Zero in RViz in joint_state_publiished and see the robot pose.
            
            # angle offset because joint 3 and 4,5,6 are not collinear.

            # offset_angle =  atan2( 0.054 , 1.5 ) == 0.036 radians
            theta3 = np.pi / 2 - (angle_b + 0.036)

            R0_3_num = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            R3_6 = R0_3_num.transpose() * R_EE_num

            
            # Symbolically, we find that
            # R3_6 = Matrix([
            #         [-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
            #         [                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
            #         [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])

            # Thus, the last 3 thetas are
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])

        # publishing the theta values
        joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
        joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()