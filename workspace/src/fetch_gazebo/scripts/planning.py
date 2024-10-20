#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib

PLANNING_GROUP = "PLANNING_GROUP"

# Actions are animations with multiple poses
# OpenAI will decide the actions
ACTION_PICK_UP_CARROT = "ACTION_PICK_UP_CARROT"
ACTION_PICK_UP_FISH = "ACTION_PICK_UP_FISH"
ACTION_PICK_UP_KNIFE = "ACTION_PICK_UP_KNIFE"
ACTION_CHOP_CARROT = "ACTION_CHOP_CARROT"
ACTION_CHOP_FISH = "ACTION_PICK_UP_FISH"
ACTION_PUT_DOWN_CARROT = "ACTION_PUT_DOWN_CARROT"
ACTION_PUT_DOWN_FISH = "ACTION_PUT_DOWN_FISH"
ACTION_PUT_DOWN_KNIFE = "ACTION_PUT_DOWN_KNIFE"

# Every action is multiple poses chained
POSE_ZERO = "POSE_ZERO"
POSE_READY_TO_GRAB_CARROT = "POSE_READY_TO_GRAB_CARROT"
POSE_READY_TO_GRAB_FISH = "POSE_READY_TO_GRAB_FISH"
POSE_READY_TO_GRAB_KNIFE = "POSE_READY_TO_GRAB_KNIFE"
POSE_ABOVE_CHOPPING_BOARD = "POSE_ABOVE_CHOPPING_BOARD"
POSE_HAND_OPEN = "POSE_HAND_OPEN"
POSE_HAND_CLOSED = "POSE_HAND_CLOSED"
POSE_CHOP_UP = "POSE_CHOP_UP"
POSE_CHOP_DOWN = "POSE_CHOP_DOWN"
POSE_DEPOSIT_KNIFE = "POSE_DEPOSIT_KNIFE"
POSE_DEPOSIT_FISH = "POSE_DEPOSIT_FISH"
POSE_DEPOSIT_CARROT = "POSE_DEPOSIT_CARROT"

ANIMATIONS_LUT = {
    ACTION_PICK_UP_CARROT: [
        {"type": POSE_ZERO, "delay": 1.0},
        {"type": POSE_HAND_OPEN, "delay": 1.0},
        {"type": POSE_READY_TO_GRAB_CARROT, "delay": 1.0},
        {"type": POSE_HAND_CLOSED, "delay": 1.0},
        {"type": POSE_ABOVE_CHOPPING_BOARD, "delay": 1.0},
        {"type": POSE_HAND_OPEN, "delay": 1.0},
    ],
    ACTION_PICK_UP_FISH: [
        {"type": POSE_ZERO, "delay": 1.0},
        {"type": POSE_HAND_OPEN, "delay": 1.0},
        {"type": POSE_READY_TO_GRAB_FISH, "delay": 1.0},
        {"type": POSE_HAND_CLOSED, "delay": 1.0},
        {"type": POSE_ABOVE_CHOPPING_BOARD, "delay": 1.0},
        {"type": POSE_HAND_OPEN, "delay": 1.0},
    ],
    ACTION_PICK_UP_KNIFE: [
        {"type": POSE_READY_TO_GRAB_KNIFE, "delay": 1.0},
        {"type": POSE_HAND_CLOSED, "delay": 1.0},
    ],
    ACTION_CHOP_CARROT: [
        {"type": POSE_ABOVE_CHOPPING_BOARD, "delay": 1.0},
        {"type": POSE_CHOP_UP, "delay": 1.0},
        {"type": POSE_CHOP_DOWN, "delay": 1.0},
        {"type": POSE_CHOP_UP, "delay": 1.0},
        {"type": POSE_CHOP_DOWN, "delay": 1.0},
        {"type": POSE_DEPOSIT_KNIFE, "delay": 1.0},
        {"type": POSE_HAND_OPEN, "delay": 1.0},
    ],
    ACTION_CHOP_FISH: [
        {"type": POSE_ABOVE_CHOPPING_BOARD, "delay": 1.0},
        {"type": POSE_CHOP_UP, "delay": 1.0},
        {"type": POSE_CHOP_DOWN, "delay": 1.0},
        {"type": POSE_CHOP_UP, "delay": 1.0},
        {"type": POSE_CHOP_DOWN, "delay": 1.0},
        {"type": POSE_DEPOSIT_KNIFE, "delay": 1.0},
        {"type": POSE_HAND_OPEN, "delay": 1.0},
    ],
    ACTION_PUT_DOWN_CARROT: [
        {"type": POSE_ABOVE_CHOPPING_BOARD, "delay": 1.0},
        {"type": POSE_DEPOSIT_CARROT, "delay": 1.0},
        {"type": POSE_HAND_OPEN, "delay": 1.0},
        {"type": POSE_ZERO, "delay": 1.0},
    ],
    ACTION_PUT_DOWN_FISH: [
        {"type": POSE_ABOVE_CHOPPING_BOARD, "delay": 1.0},
        {"type": POSE_DEPOSIT_FISH, "delay": 1.0},
        {"type": POSE_HAND_OPEN, "delay": 1.0},
        {"type": POSE_ZERO, "delay": 1.0},
    ],
    ACTION_PUT_DOWN_KNIFE: [
        {"type": POSE_ABOVE_CHOPPING_BOARD, "delay": 1.0},
        {"type": POSE_DEPOSIT_KNIFE, "delay": 1.0},
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


def main():
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


if __name__ == "__main__":
    main()
