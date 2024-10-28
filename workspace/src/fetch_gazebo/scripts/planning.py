#! /usr/bin/env python

import sys
from typing import List
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
from pydantic import BaseModel, validator, ValidationError
from openai import OpenAI

import sys
import time
import rospy
import roslib
import tf
import numpy as np
import moveit_commander
from gazebo_msgs.srv import DeleteModel
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

PLANNING_GROUP_ARM = "arm"
PLANNING_GROUP_GRIPPER = "gripper"

# Actions are animations with multiple poses
# OpenAI will decide the actions
ACTION_PREFLIGHT = "ACTION_PREFLIGHT"
ACTION_CARROT_TO_POT = "ACTION_CARROT_TO_POT"
ACTION_FISH_TO_POT = "ACTION_FISH_TO_POT"
ACTION_TOMATO_TO_POT = "ACTION_TOMATO_TO_POT"
ALLOWED_ACTIONS = [
    ACTION_CARROT_TO_POT,
    ACTION_FISH_TO_POT,
    ACTION_TOMATO_TO_POT,
]

# Every action is multiple poses chained
POSE_ABOVE_CHOPPING_BOARD = "POSE_ABOVE_CHOPPING_BOARD"
POSE_HAND_OPEN = "POSE_HAND_OPEN"
POSE_HAND_CLOSED = "POSE_HAND_CLOSED"
POSE_TYPES = [POSE_ABOVE_CHOPPING_BOARD, POSE_HAND_OPEN, POSE_HAND_CLOSED]

IK_OPERATION_HOVER = "IK_OPERATION_HOVER"
IK_OPERATION_READY_TO_GRAB = "IK_OPERATION_READY_TO_GRAB"
IK_TYPES = [IK_OPERATION_HOVER, IK_OPERATION_READY_TO_GRAB]

DISSOLVE_EVENT = "DISSOLVE_EVENT"

OBJECT_CARROT = "carrot"
OBJECT_FISH = "fish"
OBJECT_TOMATO = "tomato"
OBJECT_POT = "pot"
OBJECT_ROBOT = "fetch"

END_EFFECTOR_NAME = "wrist_roll_joint"

ANIMATIONS_LUT = {
    ACTION_PREFLIGHT: [
        {"group": PLANNING_GROUP_GRIPPER, "type": POSE_HAND_OPEN},
        {"group": PLANNING_GROUP_ARM, "type": POSE_ABOVE_CHOPPING_BOARD},
    ],
    ACTION_CARROT_TO_POT: [
        {"type": IK_OPERATION_HOVER, "target": OBJECT_CARROT},
        {"type": IK_OPERATION_READY_TO_GRAB, "target": OBJECT_CARROT},
        {"group": PLANNING_GROUP_GRIPPER, "type": POSE_HAND_CLOSED},
        {"group": PLANNING_GROUP_ARM, "type": POSE_ABOVE_CHOPPING_BOARD},
        {"type": IK_OPERATION_HOVER, "target": OBJECT_POT},
        {"group": PLANNING_GROUP_GRIPPER, "type": POSE_HAND_OPEN},
        {"type": DISSOLVE_EVENT, "target": OBJECT_CARROT},
    ],
    ACTION_FISH_TO_POT: [
        {"type": IK_OPERATION_HOVER, "target": OBJECT_FISH},
        {"type": IK_OPERATION_READY_TO_GRAB, "target": OBJECT_FISH},
        {"group": PLANNING_GROUP_GRIPPER, "type": POSE_HAND_CLOSED},
        {"group": PLANNING_GROUP_ARM, "type": POSE_ABOVE_CHOPPING_BOARD},
        {"type": IK_OPERATION_HOVER, "target": OBJECT_POT},
        {"group": PLANNING_GROUP_GRIPPER, "type": POSE_HAND_OPEN},
        {"type": DISSOLVE_EVENT, "target": OBJECT_FISH},
    ],
    ACTION_TOMATO_TO_POT: [
        {"type": IK_OPERATION_HOVER, "target": OBJECT_TOMATO},
        {"type": IK_OPERATION_READY_TO_GRAB, "target": OBJECT_TOMATO},
        {"group": PLANNING_GROUP_GRIPPER, "type": POSE_HAND_CLOSED},
        {"group": PLANNING_GROUP_ARM, "type": POSE_ABOVE_CHOPPING_BOARD},
        {"type": IK_OPERATION_HOVER, "target": OBJECT_POT},
        {"group": PLANNING_GROUP_GRIPPER, "type": POSE_HAND_OPEN},
        {"type": DISSOLVE_EVENT, "target": OBJECT_TOMATO},
    ],
}


class PoseOperator:
    def __init__(self):
        # Initialize ROS node and moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

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


class IKOperator:
    def __init__(self, arm_group):
        self.ik_solver = IK("base_link", "wrist_roll_link")
        self.ik_solver = self.stop_rolling_ffs()
        self.arm_group = arm_group

    def stop_rolling_ffs(self):
        ik_solver = self.ik_solver
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

    @staticmethod
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

    def get_ready_to_pick_with_retries(self, target_name, hover, retries=5):
        if retries <= 0:
            raise Exception(f"Could not plan {target_name},hover:{hover}")
        try:
            self.get_ready_to_pick(target_name, hover)
        except Exception:
            self.get_ready_to_pick_with_retries(target_name, hover, retries - 1)

    def get_ready_to_pick(self, target_name, hover):
        arm_group = self.arm_group
        ik_solver = self.ik_solver

        efpose = arm_group.get_current_pose().pose

        cube_result = Controller.gms_client(target_name, relative_entity_name="")
        print(cube_result.pose)

        joints = arm_group.get_current_joint_values()
        print("current joint state of the robot")
        print(arm_group.get_active_joints())
        print(joints)

        trans, qt = IKOperator.target_pose_to_ik_target(cube_result.pose, efpose, hover)

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


class Controller:
    def __init__(self):
        rospy.init_node("soup_controller", anonymous=True)
        self.pose_operator = PoseOperator()
        arm_group = self.pose_operator.group_lut[PLANNING_GROUP_ARM]
        self.ik_operator = IKOperator(arm_group)

        rospy.wait_for_service("/gazebo/delete_model")
        self.del_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    @staticmethod
    def gms_client(model_name, relative_entity_name):
        rospy.wait_for_service("/gazebo/get_model_state")
        try:
            gms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            return gms(model_name, relative_entity_name)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    @staticmethod
    def reset_model_pose(model_name):
        """
        Resets the pose of a specified model in Gazebo to the origin (0, 0, 0)
        with no rotation.

        Args:
        model_name (str): The name of the model in Gazebo to reset.

        Returns:
        bool, str: Success flag and status message from the service call.
        """
        # # Initialize the node if it hasn't been initialized yet
        # if not rospy.core.is_initialized():
        #     rospy.init_node("model_pose_resetter", anonymous=True)

        # Wait for the Gazebo service to become available
        rospy.wait_for_service("/gazebo/set_model_state")

        try:
            # Create a service proxy for setting the model state
            set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

            # Define the new model state
            model_state = ModelState()
            model_state.model_name = model_name
            model_state.pose = Pose()
            model_state.twist = Twist()
            model_state.reference_frame = "world"

            # Call the service
            response = set_state(model_state)

            # Return the response success status and status message
            return response.success, response.status_message
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False, str(e)

    def execute_operation(self, operation):
        if operation["type"] in POSE_TYPES:
            group_name = operation["group"]
            pose_name = operation["type"]
            self.pose_operator.set_pose(group_name, pose_name)
        elif operation["type"] in IK_TYPES:
            target_name = operation["target"]
            hover = operation["type"] == IK_OPERATION_HOVER
            self.ik_operator.get_ready_to_pick_with_retries(target_name, hover)
        elif operation["type"] == DISSOLVE_EVENT:
            self.del_model(operation["target"])
        else:
            raise Exception(operation)

    def execute_action(self, action_name):
        for operation in ANIMATIONS_LUT[ACTION_PREFLIGHT]:
            self.execute_operation(operation)

        self.reset_model_pose(OBJECT_ROBOT)

        for operation in ANIMATIONS_LUT[action_name]:
            self.execute_operation(operation)


# Define the allowed actions as a class attribute or global constant


class ActionPlan(BaseModel):
    actions: List[str]

    # Validator to ensure each action is in the allowed list
    @validator("actions", each_item=True)
    def check_allowed_actions(cls, v):
        if v not in ALLOWED_ACTIONS:
            raise ValueError(f"{v} is not an allowed action")
        return v


class Planner:
    def __init__(self):
        self.client = OpenAI()

    def plan_meal(self, meal_description):
        messages = [
            {
                "role": "system",
                "content": f"Generate a sequence of actions for meal preparation. Only choose from the following {','.join(ALLOWED_ACTIONS)}",
            },
            {"role": "user", "content": meal_description},
        ]

        # Call the OpenAI API
        try:
            completion = self.client.beta.chat.completions.parse(
                model="gpt-4o-2024-08-06",
                messages=messages,
                response_format=ActionPlan,
            )

            # Parse the structured output to return the list of actions
            actions = completion.choices[0].message.parsed.actions
            # Filter actions to ensure they are valid based on ALL_ACTIONS
            valid_actions = [action for action in actions if action in ALLOWED_ACTIONS]
            return valid_actions

        except Exception as e:
            print(f"Failed to retrieve actions: {e}")
            return []


def debug_loop():
    controller = Controller()

    print("Pose Operator is running. Enter action names to execute or 'exit' to quit:")
    try:
        while not rospy.is_shutdown():
            action_name = input("Enter the action name: ")
            if action_name.lower() == "exit":
                print("Exiting...")
                break
            elif action_name in ANIMATIONS_LUT:
                controller.execute_action(action_name)
            else:
                print(f"Action '{action_name}' is not defined. Try again.")
            rospy.sleep(2)
    except KeyboardInterrupt:
        print("Interrupt received, stopping the pose operator.")


def planning_loop():
    controller = Controller()
    planner = Planner()

    print(
        "Pose Operator is running. Enter meal descriptions to plan or 'exit' to quit:"
    )
    try:
        while not rospy.is_shutdown():
            meal_description = input("Enter a meal description: ")
            if meal_description.lower() == "exit":
                print("Exiting...")
                break

            actions = planner.plan_meal(meal_description)
            if actions:
                print(f"Executing actions for: {meal_description}")
                for action in actions:
                    print(f"Executing {action}")
                    controller.execute_action(action)
            else:
                print(
                    "No valid actions found for the given description. Please try again."
                )
            rospy.sleep(2)
    except KeyboardInterrupt:
        print("Interrupt received, stopping the pose operator.")


def test_vegan_meal_valid_response(planner):
    meal_description = "I want to make a vegan soup"

    actions = planner.plan_meal(meal_description)

    assert set(actions) == set(
        [
            "ACTION_CARROT_TO_POT",
            "ACTION_TOMATO_TO_POT",
        ]
    ), actions


def test_sushi_meal_valid_response(planner):
    meal_description = "I want fish soup"

    actions = planner.plan_meal(meal_description)

    assert set(actions) == set(
        [
            "ACTION_CARROT_TO_POT",
            "ACTION_TOMATO_TO_POT",
            "ACTION_FISH_TO_POT",
        ]
    ), actions


def tests():
    planner = Planner()
    test_vegan_meal_valid_response(planner)
    test_sushi_meal_valid_response(planner)


if __name__ == "__main__":
    # debug_loop()
    planning_loop()
    # tests()
