#!/usr/bin/env python3

import sys
import rospy
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
from sensor_msgs.msg import Imu

# Initialize ROS node
rospy.init_node('gui', anonymous=True)

# Global variables to store IMU data
linear_acceleration = "0.00, 0.00, 0.00"
angular_velocity = "0.00, 0.00, 0.00"
orientation = "0.00, 0.00, 0.00, 0.00"

def imu_callback(msg):
    global linear_acceleration, angular_velocity, orientation

    linear_acceleration = f"{msg.linear_acceleration.x:.2f}, {msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f}"
    angular_velocity = f"{msg.angular_velocity.x:.2f}, {msg.angular_velocity.y:.2f}, {msg.angular_velocity.z:.2f}"
    orientation = f"{msg.orientation.x:.2f}, {msg.orientation.y:.2f}, {msg.orientation.z:.2f}, {msg.orientation.w:.2f}"

if __name__ == '__main__':
    # ROS Subscriber
    rospy.Subscriber('/imu/data', Imu, imu_callback)

    # Initialize PyQt5 app
    app = QApplication(sys.argv)

    # Create main window
    window = QMainWindow()
    window.setWindowTitle('IMU Data Display')

    # Create central widget
    central_widget = QWidget()
    window.setCentralWidget(central_widget)

    # Create vertical layout
    layout = QVBoxLayout()

    # Create labels for displaying IMU data
    linear_acceleration_label = QLabel(f"Linear Acceleration (x, y, z): {linear_acceleration}")
    angular_velocity_label = QLabel(f"Angular Velocity (x, y, z): {angular_velocity}")
    orientation_label = QLabel(f"Orientation (x, y, z, w): {orientation}")

    # Add labels to the layout
    layout.addWidget(linear_acceleration_label)
    layout.addWidget(angular_velocity_label)
    layout.addWidget(orientation_label)

    # Set layout to central widget
    central_widget.setLayout(layout)

    # Show the main window
    window.show()

    # Start the ROS loop
    rospy.spin()

    # Exit the app
    sys.exit(app.exec_())
