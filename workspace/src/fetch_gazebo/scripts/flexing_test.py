#!/usr/bin/env python

import sys
import time
import rospy
import roslib
import tf
import numpy as np
import moveit_commander

from transforms3d.quaternions import mat2quat, quat2mat
from geometry_msgs.msg import PoseStamped
from trac_ik_python.trac_ik import IK

roslib.load_manifest("gazebo_msgs")
from gazebo_msgs.srv import GetModelState


def ros_quat(tf_quat):
    quat = np.zeros(4)
    quat[-1] = tf_quat[0]
    quat[:-1] = tf_quat[1:]
    return quat


def ros_qt_to_rt(rot, trans):
    qt = np.zeros((4,), dtype=np.float32)
    qt[0] = rot[3]
    qt[1] = rot[0]
    qt[2] = rot[1]
    qt[3] = rot[2]
    obj_T = np.eye(4)
    obj_T[:3, :3] = quat2mat(qt)
    obj_T[:3, 3] = trans
    return obj_T


def ros_pose_to_rt(pose):
    qarray = [0, 0, 0, 0]
    qarray[0] = pose.orientation.x
    qarray[1] = pose.orientation.y
    qarray[2] = pose.orientation.z
    qarray[3] = pose.orientation.w

    t = [0, 0, 0]
    t[0] = pose.position.x
    t[1] = pose.position.y
    t[2] = pose.position.z

    return ros_qt_to_rt(qarray, t)


def get_pose_gazebo(model_name, relative_entity_name=""):
    def gms_client(model_name, relative_entity_name):
        rospy.wait_for_service("/gazebo/get_model_state")
        try:
            gms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            return gms(model_name, relative_entity_name)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    res = gms_client(model_name, relative_entity_name)
    T_wo = ros_pose_to_rt(res.pose)
    res = gms_client(model_name="fetch", relative_entity_name="base_link")
    T_wb = ros_pose_to_rt(res.pose)
    T_wb_inv = np.linalg.inv(T_wb)
    T_bo = np.dot(T_wb_inv, T_wo)
    return T_bo


if __name__ == "__main__":
    print("Started Init")
    rospy.init_node("planning_scene_block")
    print("Finished Init")

    model_name = "demo_cube"
    T = get_pose_gazebo(model_name)
    # print("pose of the demo cube")
    # print(T)

    trans = T[:3, 3]
    qt = ros_quat(mat2quat(T[:3, :3]))

    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander("arm")

    # efpose = group.get_current_pose().pose
    # print("end-effector pose of the robot")
    # print(efpose)

    ik_solver = IK("base_link", "wrist_roll_link")
    lower_bound, upper_bound = ik_solver.get_joint_limits()
    lower_bound = list(lower_bound)
    upper_bound = list(upper_bound)
    lower_bound[0] = 0
    upper_bound[0] = 0
    ik_solver.set_joint_limits(lower_bound, upper_bound)
    # seed_state = [0.0] * ik_solver.number_of_joints

    # get the current pose of the gripper
    efpose = group.get_current_pose().pose
    position = efpose.position
    orientation = efpose.orientation
    print("end-effector pose of the robot")
    print(efpose)

    joints = group.get_current_joint_values()
    print("current joint state of the robot")
    print(group.get_active_joints())
    print(joints)
    old_joints = joints

    joints = [angle + 0.3 for angle in joints]

    group.set_joint_value_target(joints)
    plan = group.plan()
    group.go(wait=True)

    joints = group.get_current_joint_values()
    print("current joint state of the robot")
    print(group.get_active_joints())
    print(joints)

    trans = [
        efpose.position.x,
        efpose.position.y,
        efpose.position.z,
    ]

    qt = [
        efpose.orientation.x,
        efpose.orientation.y,
        efpose.orientation.z,
        efpose.orientation.w,
    ]

    # Go back to original
    seed_state = [0.0, *joints]
    sol = ik_solver.get_ik(
        seed_state, trans[0], trans[1], trans[2], qt[0], qt[1], qt[2], qt[3]
    )
    print("Solution from IK:")
    print(ik_solver.joint_names)
    print(sol)
    sol = sol[1:]

    group.set_joint_value_target(sol)
    plan = group.plan()
    group.go(wait=True)

    new_joints = group.get_current_joint_values()
    print("current joint state of the robot")
    print(old_joints)
    print(new_joints)
