#!/usr/bin/env python
import sys
import math
import rospy
import copy
import numpy as np
import tf
import moveit_commander
import helper
import motion_primitives
import yaml
import actionlib
import tilt
import regrasp
import tuck
import visualization
import robot_actions
import dynamixel

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

from std_msgs.msg import UInt16

rospy.init_node('SDI', anonymous=True)
action_name = rospy.get_param('~action_name', 'command_robotiq_action')
robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
robotiq_client.wait_for_server()
servo_pub = rospy.Publisher('servo', UInt16, queue_size=10)

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

if __name__ == '__main__':
    with open("../config/Capsule_du_config.yaml", 'r') as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    try:
        tcp_speed = config['tcp_speed']
        theta_0 = config['theta_0']
        delta_0 = config['delta_0']
        psi_regrasp = config['psi_regrasp']
        theta_tilt = config['theta_tilt']
        tuck_angle = config['tuck']
        axis =  config['axis']
        object_thickness = config['object_thickness']
        object_length = config['object_length']
        tcp2fingertip = config['tcp2fingertip']
        sim = config['sim']
        table_height_wrt_world = -0.02

        # Set TCP speed
        group.set_max_velocity_scaling_factor(tcp_speed)

        pre_pick_pose = [-0.178, -0.492, 0.416, 0.5, -0.5, -0.5, 0.5]
        pick_pose = [-0.078, 0.592, 0.302, 0.7071, -0.0, -0.7071, 0.0]
        init_pose=[-0.344, 0.538, 0.458, -0.7071, -0.0, 0.7071, 0.0]
        regrasp_pose=[0.344, -0.538, 0.346, 0.7071, -0.0, -0.7071, 0.0]


        Robotiq.goto(robotiq_client, pos=0.011, speed=config['gripper_speed'], force=config['gripper_force'], block=False)
        rospy.sleep(0.5)
        # pick and place
        motion_primitives.set_joint([151.7,-67.32,-136.40,-67.59,90.76,-28.48])
        rospy.sleep(1)

        motion_primitives.set_pose_relative([0,0,0.0025])

        servo_pub.publish(160)
        rospy.sleep(1)
        dynamixel.set_position(230)
        rospy.sleep(2)
        dynamixel.set_position(100)
        rospy.sleep(0.5)




        # read position from real robot.

        p = group.get_current_pose().pose
        trans_tool0 = [p.position.x, p.position.y, p.position.z]
        rot_tool0 = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        T_wg = tf.TransformerROS().fromTranslationRotation(trans_tool0, rot_tool0)
        P_g_center = [tcp2fingertip+object_length-delta_0, 0, 0, 1]
        P_w_center = np.matmul(T_wg, P_g_center)
        center = [P_w_center[0], P_w_center[1], P_w_center[2]]

        # Tilt
        tilt.tilt(center, axis, int(90-theta_0), tcp_speed)
        rospy.sleep(2)

        # Regrasp

        motion_primitives.set_pose_relative([0,0.002,0])
        width = regrasp.regrasp(np.multiply(axis, -1), int(psi_regrasp), tcp_speed)
        rospy.sleep(0.5)

        # Tilt
        tilt.tilt(center, axis, int(theta_tilt), tcp_speed)
        rospy.sleep(0.5)
        print width



        # Second regrasp_pose
        motion_primitives.set_pose_relative([0,0.001 *10,-0.001 *0])
        regrasp.regrasp_test(np.multiply(axis, -1), int(tuck_angle), tcp_speed, width-0.002, theta_init=theta_0+psi_regrasp-theta_tilt+1, psi_init=psi_regrasp)

        motion_primitives.set_pose_relative([0,0,0.1])


        servo_pub.publish(0)
        rospy.sleep(0.5)
        servo_pub.publish(100)

        #rospy.spin()

    except rospy.ROSInterruptException: pass
