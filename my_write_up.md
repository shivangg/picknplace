## Project: Kinematics Pick & Place

### 1. Kinematics Analysis

Used the kr210.xacro.urdf file to evaluate the D-H parameters. 


The D-H Parameter Table 

Link(i)  |	alpha( i-1 ) 	|	a( i-1 )  |	 d( i-1 )  |  theta( i )
---		 |	---				|	---	      |	 ---       |  ---	
1		|	0  				|	0  			|     0.75     |   q1
2		|	-pi/2			|	0.35 		|     0 	   |   q2 - pi/2
3		|	0  				|	1.25 		|     0 	   |   q3
4		|	-pi/2			|	-0.054 		|     1.5      |   q4
5		|	pi/2			|	0 			|     0        |   q5
6		|	-pi/2			|	0 	 		|     0    	   |   q6
G		|	0				|	0 	 		|     0.303    |   0


[](GIF image)


### Find a1, a2, a3, d4, dg

1. First find the 3D coordinates of the six joints and the gripper using the `kr210.urdf.xacro` file. The XYZ reading are relative to the parent link so we take the cumulative sum of the readings.

[](pic)  

2. Make a python dictionary of known D-H parameters `s`.

3. Compose the homogenous transform matrices using the D-H parameters for the calculating the transform from 0 to i(i = 1,2,3,4,5,6,G).

4. Correct the orientation of the gripper.

5. Now, what is left is to solve the inverse kinematics of the arm. For help in referred to the IK example.

#### This is divided into 2 parts: Inverse Position and Inverse Orientation

1. `Inverse Position` is calculated by solving the joints `q1`, `q2` and `q3`.
2. `Inverse Orientation` is calculated using the joints `q4`, `q5` and `q6`.

First calculated WC from EE coordinates using:

`WC = EE - (0.303) * R_EE_num[:, 2]`

 where `R_EE_num` is the transform from `base_frame` to the End effector along with 

 1. Corrected gripper orientation
 2. Substituted with correct Roll, Pitch and Yaw values.

 0.303 is d(gripper).

For calculation of joint angles

Theta1

theta1 = atan2( yc , xc)

where yx and xc are coordinates of the WC. 

For calculating we look in the Z-X frame of reference.

Theta2

theta2 = pi / 2  -  angle_a - phi

Theta3

theta3 = pi / 2 - offset_angle - angle_b

angle_a , angle_b as the figure.

`offset_angle` is introduced because the centre points of j3 and j4, j5 and j6 are *not* collinear. 









Q. Who calculates the path displayed when the RViz when the text shows "Displaying path to dropOff"

How to display the angles in Rviz?
rostopic echo joint_states
maybe print the thetas. the last of them is the end effector.

