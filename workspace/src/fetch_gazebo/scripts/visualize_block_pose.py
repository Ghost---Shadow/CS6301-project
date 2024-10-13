#!/usr/bin/env python

"""
CS 6301 Homework 2 Programming
Transformation
"""

import rospy
import roslib
import tf
import numpy as np
from transforms3d.quaternions import mat2quat, quat2mat
from numpy.linalg import inv

roslib.load_manifest("gazebo_msgs")
from gazebo_msgs.srv import GetModelState


# quaternion in ROS is with format (xyzw)
def ros_quat(tf_quat):  # wxyz -> xyzw
    quat = np.zeros(4)
    quat[-1] = tf_quat[0]
    quat[:-1] = tf_quat[1:]
    return quat


# Convert quaternion and translation to a 4x4 tranformation matrix
# See Appendix B.3 in Lynch and Park, Modern Robotics for the definition of quaternion
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


# Convert a ROS pose message to a 4x4 tranformation matrix
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


# Query pose of frames from the Gazebo environment
def get_pose_gazebo(model_name, relative_entity_name=""):

    def gms_client(model_name, relative_entity_name):
        rospy.wait_for_service("/gazebo/get_model_state")
        try:
            gms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            resp1 = gms(model_name, relative_entity_name)
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    # Query the object pose in Gazebo world T_wo
    res = gms_client(model_name, relative_entity_name)
    T_wo = ros_pose_to_rt(res.pose)

    # Query Fetch base link pose in Gazebo world T_wb
    res = gms_client(model_name="fetch", relative_entity_name="base_link")
    T_wb = ros_pose_to_rt(res.pose)

    # Compute the inverse of the Fetch base link transformation matrix
    T_bw = inv(T_wb)

    # Compute the object pose in robot base link frame T_bo
    T_bo = np.dot(T_bw, T_wo)

    return T_bo


if __name__ == "__main__":
    """
    Main function to run the code
    """

    # query the demo cube pose
    model_name = "demo_cube"
    T = get_pose_gazebo(model_name)
    print(T)

    # translation
    trans = T[:3, 3]

    # quaternion in ros
    qt = ros_quat(mat2quat(T[:3, :3]))

    # broadcast the tf frame for visualization in rviz
    rospy.init_node("tf_broadcaster")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # publish the transformation from base_link to model_name
        br.sendTransform(trans, qt, rospy.Time.now(), model_name, "base_link")
        rate.sleep()
        print("publish tf ", model_name)
