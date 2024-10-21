#! /usr/bin/env python

import sys
from typing import List
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
from pydantic import BaseModel, validator, ValidationError
from openai import OpenAI

PLANNING_GROUP = "PLANNING_GROUP"

# Actions are animations with multiple poses
# OpenAI will decide the actions
ACTION_CARROT_TO_CHOPPING_BOARD = "ACTION_CARROT_TO_CHOPPING_BOARD"
ACTION_FISH_TO_CHOPPING_BOARD = "ACTION_FISH_TO_CHOPPING_BOARD"
ACTION_CHOP_AND_PUT_DOWN_KNIFE = "ACTION_CHOP_AND_PUT_DOWN_KNIFE"
ACTION_DELIVER_CHOPPED_ITEM = "ACTION_DELIVER_CHOPPED_ITEM"

# Every action is multiple poses chained
POSE_ZERO = "POSE_ZERO"
POSE_READY_TO_GRAB_CARROT = "POSE_READY_TO_GRAB_CARROT"
POSE_READY_TO_GRAB_FISH = "POSE_READY_TO_GRAB_FISH"
POSE_KNIFE_LOCATION = "POSE_KNIFE_LOCATION"
POSE_ABOVE_CHOPPING_BOARD = "POSE_ABOVE_CHOPPING_BOARD"
POSE_READY_TO_GRAB_CHOPPED_ITEM = "POSE_READY_TO_GRAB_CHOPPED_ITEM"
POSE_HAND_OPEN = "POSE_HAND_OPEN"
POSE_HAND_CLOSED = "POSE_HAND_CLOSED"
POSE_CHOP_UP = "POSE_CHOP_UP"
POSE_CHOP_DOWN = "POSE_CHOP_DOWN"
POSE_DINNER_PLATE_LOCATION = "POSE_DINNER_PLATE_LOCATION"


ANIMATIONS_LUT = {
    ACTION_CARROT_TO_CHOPPING_BOARD: [
        {"type": POSE_ZERO, "delay": 1.0},
        {"type": POSE_HAND_OPEN, "delay": 1.0},
        {"type": POSE_READY_TO_GRAB_CARROT, "delay": 1.0},
        {"type": POSE_HAND_CLOSED, "delay": 1.0},
        {"type": POSE_ABOVE_CHOPPING_BOARD, "delay": 1.0},
        {"type": POSE_HAND_OPEN, "delay": 1.0},
    ],
    ACTION_FISH_TO_CHOPPING_BOARD: [
        {"type": POSE_ZERO, "delay": 1.0},
        {"type": POSE_HAND_OPEN, "delay": 1.0},
        {"type": POSE_READY_TO_GRAB_FISH, "delay": 1.0},
        {"type": POSE_HAND_CLOSED, "delay": 1.0},
        {"type": POSE_ABOVE_CHOPPING_BOARD, "delay": 1.0},
        {"type": POSE_HAND_OPEN, "delay": 1.0},
    ],
    ACTION_CHOP_AND_PUT_DOWN_KNIFE: [
        {"type": POSE_KNIFE_LOCATION, "delay": 1.0},
        {"type": POSE_HAND_CLOSED, "delay": 1.0},
        {"type": POSE_ABOVE_CHOPPING_BOARD, "delay": 1.0},
        {"type": POSE_CHOP_UP, "delay": 1.0},
        {"type": POSE_CHOP_DOWN, "delay": 1.0},
        {"type": POSE_CHOP_UP, "delay": 1.0},
        {"type": POSE_CHOP_DOWN, "delay": 1.0},
        {"type": POSE_KNIFE_LOCATION, "delay": 1.0},
        {"type": POSE_HAND_OPEN, "delay": 1.0},
    ],
    ACTION_DELIVER_CHOPPED_ITEM: [
        {"type": POSE_HAND_OPEN, "delay": 1.0},
        {"type": POSE_ABOVE_CHOPPING_BOARD, "delay": 1.0},
        {"type": POSE_READY_TO_GRAB_CHOPPED_ITEM, "delay": 1.0},
        {"type": POSE_HAND_CLOSED, "delay": 1.0},
        {"type": POSE_ABOVE_CHOPPING_BOARD, "delay": 1.0},
        {"type": POSE_DINNER_PLATE_LOCATION, "delay": 1.0},
        {"type": POSE_HAND_OPEN, "delay": 1.0},
        {"type": POSE_ZERO, "delay": 1.0},
    ],
}


class PoseOperator:
    def __init__(self, planning_group):
        # Initialize ROS node and moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("node_set_redefined_pose", anonymous=True)

        # Core components of the robot interface
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(planning_group)

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

        # Optional: Log key information
        rospy.loginfo(f"Robot initialized with planning group: {planning_group}")

    def set_pose(self, pose_name):
        self.group.set_named_target(pose_name)
        plan = self.group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self.execute_trajectory_client.send_goal(goal)
        self.execute_trajectory_client.wait_for_result()

    def execute_action(self, action_name):
        """
        Execute a series of poses based on a predefined action.

        Args:
        action_name (str): The name of the action to execute as defined in ANIMATIONS_LUT.
        """
        if action_name in ANIMATIONS_LUT:
            for step in ANIMATIONS_LUT[action_name]:
                pose_name = step["type"]
                delay = step["delay"]
                rospy.loginfo(
                    f"Executing pose: {pose_name} with a delay of {delay} seconds"
                )
                self.set_pose(pose_name)
                rospy.sleep(2)
        else:
            rospy.loginfo(f"Action {action_name} is not defined.")


# Define the allowed actions as a class attribute or global constant
ALLOWED_ACTIONS = [
    ACTION_CARROT_TO_CHOPPING_BOARD,
    ACTION_FISH_TO_CHOPPING_BOARD,
    ACTION_CHOP_AND_PUT_DOWN_KNIFE,
    ACTION_DELIVER_CHOPPED_ITEM,
]


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
    pose_operator = PoseOperator(PLANNING_GROUP)

    print("Pose Operator is running. Enter action names to execute or 'exit' to quit:")
    try:
        while not rospy.is_shutdown():
            action_name = input("Enter the action name: ")
            if action_name.lower() == "exit":
                print("Exiting...")
                break
            elif action_name in ANIMATIONS_LUT:
                pose_operator.execute_action(action_name)
            else:
                print(f"Action '{action_name}' is not defined. Try again.")
            rospy.sleep(2)
    except KeyboardInterrupt:
        print("Interrupt received, stopping the pose operator.")


def planning_loop():
    pose_operator = PoseOperator(PLANNING_GROUP)
    planner = Planner()  # Assuming Planner class is already defined and imported

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
                    pose_operator.execute_action(action)
            else:
                print(
                    "No valid actions found for the given description. Please try again."
                )
            rospy.sleep(2)
    except KeyboardInterrupt:
        print("Interrupt received, stopping the pose operator.")


def test_vegan_meal_valid_response(planner):
    meal_description = "I want to make a vegan salad"

    actions = planner.plan_meal(meal_description)

    assert actions == [
        "ACTION_CARROT_TO_CHOPPING_BOARD",
        "ACTION_CHOP_AND_PUT_DOWN_KNIFE",
        "ACTION_DELIVER_CHOPPED_ITEM",
    ], actions


def test_sushi_meal_valid_response(planner):
    meal_description = "I want sushi"

    actions = planner.plan_meal(meal_description)

    assert actions == [
        "ACTION_FISH_TO_CHOPPING_BOARD",
        "ACTION_CHOP_AND_PUT_DOWN_KNIFE",
        "ACTION_DELIVER_CHOPPED_ITEM",
    ], actions


def tests():
    planner = Planner()
    test_vegan_meal_valid_response(planner)
    test_sushi_meal_valid_response(planner)


if __name__ == "__main__":
    # debug_loop()
    # planning_loop()
    tests()
