#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty, EmptyResponse
from rover.srv import ToggleIMURepresentation
import tf.transformations as tf

class IMURepresentationServer:
    def __init__(self):
        rospy.init_node('imu_representation_server')

        # Subscribe to the IMU topic
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        # Define the service
        self.service = rospy.Service('toggle_imu_representation', ToggleIMURepresentation, self.handle_toggle_imu_representation)

        # Variable to store the current representation
        self.current_representation = "quaternion"

    def imu_callback(self, msg):
        if self.current_representation == "quaternion":
            orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        else:  # Assume Euler for the sake of example
            orientation = tf.euler_from_quaternion((msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))

        # Here you can publish or do something with the converted orientation
        rospy.loginfo(f"Current orientation representation: {self.current_representation}, Orientation: {orientation}")

    def handle_toggle_imu_representation(self, req):
        if self.current_representation == "quaternion":
            self.current_representation = "euler"
        else:
            self.current_representation = "quaternion"
        
        rospy.loginfo(f"Switched to {self.current_representation} representation")
        return EmptyResponse()

if __name__ == '__main__':
    imu_representation_server = IMURepresentationServer()
    rospy.spin()
