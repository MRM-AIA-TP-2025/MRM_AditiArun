#!/usr/bin/env python3

import rospy
from rover.srv import ToggleIMURepresentation
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

def quaternion_to_euler(req):
    quaternion = (req.x, req.y, req.z, req.w)
    euler = euler_from_quaternion(quaternion)
    return euler[0], euler[1], euler[2]

def euler_to_quaternion(req):
    euler = (req.roll, req.pitch, req.yaw)
    quaternion = quaternion_from_euler(*euler)
    return quaternion[0], quaternion[1], quaternion[2], quaternion[3]

def imu_representation_server():
    rospy.init_node('imu_representation_server')

    # Create the quaternion to euler service
    rospy.Service('quaternion_to_euler', ToggleIMURepresentation, quaternion_to_euler)

    # Create the euler to quaternion service
    rospy.Service('euler_to_quaternion', ToggleIMURepresentation, euler_to_quaternion)

    rospy.spin()

if __name__ == "__main__":
    imu_representation_server()
