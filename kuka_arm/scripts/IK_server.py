#!/usr/bin/env python
import math
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np
from numpy import array
from sympy.matrices import Matrix

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:


        # Creating symbols for theta, alpha, "d", "a"
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #theta
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5 , alpha6 = symbols('alpha0:7')


	# Creating the Modified DH parameters
	s = {alpha0:     0,  a0:     0,  d1:   0.75,
             alpha1: -pi/2,  a1:  0.35,  d2:      0, q2:  q2-pi/2,
             alpha2:     0,  a2:  1.25,  d3:      0,
             alpha3: -pi/2,  a3:-0.054,  d4:    1.5,
             alpha4:  pi/2,  a4:     0,  d5:      0,
             alpha5: -pi/2,  a5:     0,  d6:      0,
             alpha6:     0,  a6:     0,  d7:  0.303, q7:     0}

	#the matrix function to return the transformation matrix for joint i
        def matrix(alpha, a, d, q):
            answer = Matrix([[             cos(q),            -sin(q),            0,              a],
                             [  sin(q)*cos(alpha),  cos(q)*cos(alpha),  -sin(alpha),  -sin(alpha)*d],
                             [  sin(q)*sin(alpha),  cos(q)*sin(alpha),   cos(alpha),   cos(alpha)*d],
                             [                  0,                  0,            0,              1]])
            return answer


        # the rotational functions to rotate any frame by x, y or z axis by specific angle
        def rot_x(angle):
            R_x = Matrix([[   1,                0,               0,     0],
                        [     0,       cos(angle),     -sin(angle),     0],
                        [     0,       sin(angle),      cos(angle),     0],
                        [     0,                0,               0,     1]])
            return R_x

        def rot_y(angle):
            R_y = Matrix([[   cos(angle),                0,    sin(angle),     0],
                         [             0,                1,             0,     0],
                         [   -sin(angle),                0,    cos(angle),     0],
                         [             0,                0,             0,     1]])
            return R_y

        def rot_z(angle):
            R_z = Matrix([[      cos(angle),      -sin(angle),          0,      0],
                         [       sin(angle),       cos(angle),          0,      0],
                         [                0,                0,          1,      0],
                         [                0,                0,          0,      1]])
            return R_z

        # R-corr to compensate the difference between DH parameters and Gazebo
        R_corr= simplify(rot_z(np.pi) * rot_y(-np.pi/2))



        # Initialize service response
        joint_trajectory_list = []
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

            # first to get the position of EE and its orientation
	    EE_position = Matrix([[px],[py],[pz]])
            R_EE = rot_z(yaw)[0:3,0:3] * rot_y(pitch)[0:3,0:3] * rot_x(roll)[0:3,0:3] *R_corr[0:3,0:3]

            #using the wrist center position to calculate the first 3 angles
            WC = EE_position - 0.303*R_EE[:,2]
            # theta 1 calculated using WC-x position and WC-y position
            theta_1 = atan2(WC[1],WC[0])
            #print('theta 1 = ', theta_1)

            # for theta 2 we use the corrected distance-x and the corrected distance-y to calculate branch_B length
            WX_new = sqrt(pow(WC[0],2) + pow(WC[1],2)) - 0.35
            WZ_new = WC[2] - 0.75
            branch_B = sqrt(pow(WZ_new,2) + pow(WX_new,2))
            # as C & A are fixed distances then we can calculate aaangle_a and angle_Q
            C = 1.25
            A = 1.5
            angle_a = math.acos(( pow(branch_B,2) + pow(C,2) - pow(A,2) ) / ( 2 * branch_B * C ))
            angle_Q = atan2(WZ_new,WX_new)
            # then theta 2 as follows :-
            theta_2 = np.pi/2 - angle_a - angle_Q

            #print ('theta 2 = ', theta_2)


            # to get theta 3 we have to calculate angle_b first as follows:-
            angle_b = math.acos((pow(C,2) + pow(A,2) - pow(branch_B,2)) / (2 * C * A))
            theta_3 = np.pi/2 - angle_b - 0.03598 # 0.03598 is fixed angle = atan2(0.054,1.5)


            # by using the T0_3 matrix to get R0_3 we can calculate R3_6 as follows :-

	    Rrpy = R_EE

            T0_1 = matrix(alpha=alpha0, a=a0, d=d1, q=q1)
            T1_2 = matrix(alpha=alpha1, a=a1, d=d2, q=q2)
            T2_3 = matrix(alpha=alpha2, a=a2, d=d3, q=q3)

            T0_1 = T0_1.subs(s)
            T1_2 = T1_2.subs(s)
            T2_3 = T2_3.subs(s)

            T0_2 = simplify(T0_1 * T1_2)
            T0_3 = simplify(T0_2 * T2_3)

            R0_3 = T0_3.evalf(subs={q1: theta_1, q2: theta_2, q3: theta_3})[0:3,0:3]

            R3_6 = R0_3.inv("LU") * Rrpy
            # using R3_6 to obtain the last 3 angles
            theta_4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta_5 = atan2(sqrt(R3_6[0, 2]*R3_6[0, 2]+R3_6[2, 2]*R3_6[2, 2]), R3_6[1, 2])
            theta_6 = atan2(-R3_6[1, 1], R3_6[1, 0])


	    joint_trajectory_point.positions = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]
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
