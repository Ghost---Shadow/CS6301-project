#! /usr/bin/env python

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose


def get_model_pose(model_name):
    rospy.wait_for_service("/gazebo/get_model_state")
    try:
        get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        resp = get_model_state(model_name, "world")
        return resp.pose
    except rospy.ServiceException as e:
        print("Get model state service call failed: %s" % e)
        return None


def spawn_model(model_name, model_xml, pose):
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    try:
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        spawn_model(model_name, model_xml, "", pose, "world")
    except rospy.ServiceException as e:
        print("Spawn model service call failed: %s" % e)


def delete_model(model_name):
    rospy.wait_for_service("/gazebo/delete_model")
    try:
        del_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        del_model(model_name)
    except rospy.ServiceException as e:
        print("Delete model service call failed: %s" % e)


if __name__ == "__main__":
    rospy.init_node("model_manager", anonymous=True)

    current_model = "pot"
    new_model_name = "pot"
    new_model_file = open(
        "/home/ros/workspace/src/fetch_gazebo/models/pot/model.sdf", "r"
    ).read()

    # Get current pose of the model before deleting
    current_pose = get_model_pose(current_model)
    if current_pose is not None:
        delete_model(current_model)
        spawn_model(new_model_name, new_model_file, current_pose)
    else:
        print(f"Failed to get pose for model {current_model}")
