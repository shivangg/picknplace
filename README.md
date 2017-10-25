## Project: Kinematics Pick & Place

### 1. Kinematics Analysis

Here is the [YouTube video](https://youtu.be/yYx3GbzMq0A) showing the pick and plcace operation using the *KR210 robotic arm*

Used the kr210.xacro.urdf file to evaluate the D-H parameters. 

![The joints with gripper ][frame]


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


![measurement figure][measure]


### Find a1, a2, a3, d4, dg

1. First find the 3D coordinates of the six joints and the gripper using the `kr210.urdf.xacro` file. The XYZ reading are relative to the parent link so we take the cumulative sum of the readings.

[](pic)  

2. Make a python dictionary of known D-H parameters `s`.

3. Compose the homogeneous transform matrices using the D-H parameters for the calculating the transform from 0 to i(i = 1,2,3,4,5,6,G).

4. Correct the orientation of the gripper.

5. Now, what is left is to solve the inverse kinematics of the arm. For help in referred to the IK example.

#### This is divided into 2 parts: Inverse Position and Inverse Orientation

1. `Inverse Position` is calculated by solving the joints `q1`, `q2` and `q3`.
2. `Inverse Orientation` is calculated using the joints `q4`, `q5` and `q6`.

![Figure for IK ][IK_fig]

First calculated WC from EE coordinates using:

`WC = EE - (0.303) * R_EE_num[:, 2]`

 where `R_EE_num` is the transform from `base_frame` to the End effector along with 

 1. Corrected gripper orientation
 2. Substituted with correct Roll, Pitch and Yaw values.

 0.303 is d(gripper).



## For calculation of joint angles

### Theta1

![joint 1 Angle][j1]

theta1 = atan2( yc , xc)

where yx and xc are coordinates of the WC. 

For calculating we look in the Z-X frame of reference.

### Theta2

For calculating `theta2`, because the side_a, side_b and side_c is known, _cosine rule_ is used to find the angles of the triangle in the figure below. 

![joint 2 Angle][j2] 

```python
theta2 = pi / 2  -  angle_a - phi
```

### Theta3

The joint angle `theta3` is also calculated using the angles found using the _cosine rule_.

![joint 3 Angle][j3] 

```python
theta3 = pi / 2 - offset_angle - angle_b
```

`angle_a` , `angle_b` as the figure.

`offset_angle` is introduced because the centre points of j3 and j4, j5 and j6 are *not* collinear. 

## For calculating theta4, theta5 and theta6

These joint angles can be calculated by observing the *symbolic representation* of `R3_6` and manipulating its elements. It can out to be:

```python
R3_6 = Matrix([
        [-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
        [                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
        [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])


```
## theta4

tan(theta4) = R3_6[2,2] / -R3_6[0,2]

## theta5

tan(theta5) = sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2,2] * R3_6[2,2]) /  R3_6[1,2])

## theta6

tan(theta6) = -R3_6[1,1] / R3_6[1,0]

#### After finding out these angles, they were published to the RViz which moves the robotics arm KR210 also simulated in the Gazebo environment.

#### Improvements
This project can be improved by making a program that can produce closed-form inverse kinematics solutions automagically from the URDF file of the robot. Something like [ik_fast](http://openrave.org/docs/0.8.2/openravepy/ikfast/).

#### Where it might Fail
This will fail to calculate solutions if the point to be reached is at a greater distance than the workspace of the robot, although in real cases extenders might be used at the cost of payload capacity. Mobile manipulators is also another option for pick and placement of far away objects.



[//]: # (Image References)

[coordinates]: ./misc_images/EquationsPics/XYZrpyJoints.jpg
[j1]: ./misc_images/EquationsPics/IK_calc_J1.jpg
[j2]: ./misc_images/EquationsPics/IK_calc_J2.jpg
[j3]: ./misc_images/EquationsPics/IK_calc_J3.jpg
[IK_fig]: ./misc_images/EquationsPics/IK_calcFig.jpg
[measure]: ./misc_images/EquationsPics/measurements.jpg
[frame]: ./misc_images/EquationsPics/jointFrame.jpg