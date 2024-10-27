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
import moveit_msgs
import actionlib
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Quaternion, Twist

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


def gms_client(model_name, relative_entity_name):
    rospy.wait_for_service("/gazebo/get_model_state")
    try:
        gms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        return gms(model_name, relative_entity_name)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def get_pose_gazebo(model_name, relative_entity_name=""):
    res = gms_client(model_name, relative_entity_name)
    T_wo = ros_pose_to_rt(res.pose)
    res = gms_client(model_name="fetch", relative_entity_name="base_link")
    T_wb = ros_pose_to_rt(res.pose)
    T_wb_inv = np.linalg.inv(T_wb)
    T_bo = np.dot(T_wb_inv, T_wo)
    return T_bo


PLANNING_GROUP_ARM = "arm"
PLANNING_GROUP_GRIPPER = "gripper"

POSE_ABOVE_CHOPPING_BOARD = "POSE_ABOVE_CHOPPING_BOARD"
POSE_ZERO = "POSE_ZERO"

POSE_HAND_OPEN = "POSE_HAND_OPEN"
POSE_HAND_CLOSED = "POSE_HAND_CLOSED"


class PoseOperator:
    def __init__(self):
        # Initialize ROS node and moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node("node_set_redefined_pose", anonymous=True)

        # Core components of the robot interface
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_lut = {
            PLANNING_GROUP_ARM: moveit_commander.MoveGroupCommander(PLANNING_GROUP_ARM),
            PLANNING_GROUP_GRIPPER: moveit_commander.MoveGroupCommander(
                PLANNING_GROUP_GRIPPER
            ),
        }

        # Trajectory visualization and execution
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1,
        )

        # Create an action client for executing trajectories
        self.execute_trajectory_client = actionlib.SimpleActionClient(
            "execute_trajectory", moveit_msgs.msg.ExecuteTrajectoryAction
        )
        self.execute_trajectory_client.wait_for_server()

    def set_pose(self, group_name, pose_name):
        group = self.group_lut[group_name]

        # Set the target pose by name
        group.set_named_target(pose_name)

        # Plan to the given pose
        plan = group.plan()[1]

        # Execute the plan
        success = group.execute(plan, wait=True)
        group.stop()  # Ensure that there is no residual movement

        # Clear targets after execution
        group.clear_pose_targets()

        # Check if the plan was successfully executed
        if not success:
            rospy.logerr("Failed to execute plan to {}".format(pose_name))
        else:
            rospy.loginfo("Successfully executed plan to {}".format(pose_name))

        return success


def loud_print(s):
    print("-" * 80)
    print(s)
    print("-" * 80)


def stop_rolling_ffs(ik_solver):
    lower_bound, upper_bound = ik_solver.get_joint_limits()
    lower_bound = list(lower_bound)
    upper_bound = list(upper_bound)
    lower_bound[0] = 0  # torso_lift_joint
    upper_bound[0] = 0  # torso_lift_joint
    for i, joint_name in enumerate(ik_solver.joint_names):
        if joint_name in ["upperarm_roll_joint", "forearm_roll_joint"]:
            lower_bound[i] = -1e-1
            upper_bound[i] = 1e-1
    print("rolling limits")
    print(ik_solver.joint_names)
    print(lower_bound)
    print(upper_bound)
    ik_solver.set_joint_limits(lower_bound, upper_bound)
    return ik_solver


def reset_model_pose(model_name):
    """
    Resets the pose of a specified model in Gazebo to the origin (0, 0, 0)
    with no rotation.

    Args:
    model_name (str): The name of the model in Gazebo to reset.

    Returns:
    bool, str: Success flag and status message from the service call.
    """
    # Initialize the node if it hasn't been initialized yet
    if not rospy.core.is_initialized():
        rospy.init_node("model_pose_resetter", anonymous=True)

    # Wait for the Gazebo service to become available
    rospy.wait_for_service("/gazebo/set_model_state")

    try:
        # Create a service proxy for setting the model state
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        # Define the new model state
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = Pose()  # This defaults to position (0,0,0) and no rotation
        model_state.twist = Twist()  # No linear or angular velocity
        model_state.reference_frame = "world"  # Pose relative to the world frame

        # Call the service
        response = set_state(model_state)

        # Return the response success status and status message
        return response.success, response.status_message
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False, str(e)


def target_pose_to_ik_target(target_pose, end_effector_pose, hover):
    cube_global_position = [
        target_pose.position.x,
        target_pose.position.y,
        target_pose.position.z,
    ]
    if hover:
        offset = 0.25
    else:
        offset = 0.175

    trans = [
        cube_global_position[0],
        cube_global_position[1],
        cube_global_position[2] + offset,
    ]

    qt = [
        end_effector_pose.orientation.x,
        end_effector_pose.orientation.y,
        end_effector_pose.orientation.z,
        end_effector_pose.orientation.w,
    ]

    return trans, qt


def get_ready_to_pick(arm_group, target_name, hover):
    efpose = arm_group.get_current_pose().pose
    # position = efpose.position
    # orientation = efpose.orientation
    print("end-effector pose of the robot")
    print(efpose)

    # IK solver
    ik_solver = IK("base_link", "wrist_roll_link")
    ik_solver = stop_rolling_ffs(ik_solver)

    cube_result = gms_client(target_name, relative_entity_name="")
    print(cube_result.pose)

    joints = arm_group.get_current_joint_values()
    print("current joint state of the robot")
    print(arm_group.get_active_joints())
    print(joints)

    trans, qt = target_pose_to_ik_target(cube_result.pose, efpose, hover)

    seed_state = [0.0, *joints]
    sol = ik_solver.get_ik(
        seed_state, trans[0], trans[1], trans[2], qt[0], qt[1], qt[2], qt[3]
    )
    print("Solution from IK:")
    print(ik_solver.joint_names)
    print(sol)
    sol = sol[1:]

    loud_print("Moving")
    arm_group.set_joint_value_target(sol)
    plan = arm_group.plan()
    arm_group.go(wait=True)


def hover_then_snatch(pose_operator, target_name):
    arm_group = pose_operator.group_lut[PLANNING_GROUP_ARM]

    get_ready_to_pick(arm_group, target_name, hover=True)
    get_ready_to_pick(arm_group, target_name, hover=False)

    pose_operator.set_pose(PLANNING_GROUP_GRIPPER, POSE_HAND_CLOSED)


def drop_in_pot(pose_operator, pot_name):
    arm_group = pose_operator.group_lut[PLANNING_GROUP_ARM]

    pose_operator.set_pose(PLANNING_GROUP_ARM, POSE_ABOVE_CHOPPING_BOARD)
    get_ready_to_pick(arm_group, pot_name, hover=True)
    pose_operator.set_pose(PLANNING_GROUP_GRIPPER, POSE_HAND_OPEN)


if __name__ == "__main__":
    print("Started Init")
    rospy.init_node("touch_that_cube")
    print("Finished Init")

    target_names = [
        "carrot",
        "salt",
        "fish",
    ]
    SELF_NAME = "fetch"
    POT_NAME = "pot"

    pose_operator = PoseOperator()

    for target_name in target_names:
        loud_print("Arm above board")
        pose_operator.set_pose(PLANNING_GROUP_GRIPPER, POSE_HAND_OPEN)
        pose_operator.set_pose(PLANNING_GROUP_ARM, POSE_ABOVE_CHOPPING_BOARD)
        reset_model_pose("fetch")

        hover_then_snatch(pose_operator, target_name)
        drop_in_pot(pose_operator, POT_NAME)
