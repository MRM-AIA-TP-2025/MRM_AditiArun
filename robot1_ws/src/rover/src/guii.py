#!/usr/bin/env python3

import sys
import rospy
import cv2
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QPushButton, QGridLayout
from PyQt5.QtGui import QImage, QPixmap
from sensor_msgs.msg import Image, Imu, NavSatFix, LaserScan
from cv_bridge import CvBridge
from threading import Thread
import numpy as np
from rover.srv import ToggleIMURepresentation
import csv


def quaternion_to_euler_client(x, y, z, w):
    rospy.wait_for_service('quaternion_to_euler')
    try:
        quaternion_to_euler = rospy.ServiceProxy('quaternion_to_euler', ToggleIMURepresentation)
        response = quaternion_to_euler(x, y, z, w)
        return response.roll, response.pitch, response.yaw
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

class RoverDisplay(QMainWindow):
    def __init__(self):
        super().__init__()

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.layout = QGridLayout()

        self.video_label = QLabel(self)
        self.imu_label = QLabel(self)
        self.gps_label = QLabel(self)
        self.obstacle_label = QLabel(self)
        

        self.screenshot_button = QPushButton('Screenshot', self)
        self.screenshot_button.clicked.connect(self.capture_screenshot)

        self.record_button = QPushButton('Record', self)
        self.record_button.clicked.connect(self.toggle_record)

        self.csv_button = QPushButton('CSV', self)
        self.csv_button.clicked.connect(self.csv_record)

        self.linear_acceleration_label = QLabel("Linear Acceleration (x, y, z): ")
        self.angular_velocity_label = QLabel("Angular Velocity (x, y, z): ")
        self.orientation_label = QLabel("Orientation (x, y, z, w): ")

        # Add labels to the layout
        self.layout.addWidget(self.linear_acceleration_label,0,1)
        self.layout.addWidget(self.angular_velocity_label,1,1)
        self.layout.addWidget(self.orientation_label,2,1)

        self.layout.addWidget(self.video_label, 0, 0, 5, 1)
        #self.layout.addWidget(self.imu_label, 0, 1)
        self.layout.addWidget(self.gps_label, 3, 1)
        self.layout.addWidget(self.obstacle_label, 4, 1)
        self.layout.addWidget(self.screenshot_button, 5, 1)
        self.layout.addWidget(self.record_button, 6, 1)

        self.toggle_button = QPushButton("Toggle Orientation")
        self.toggle_button.clicked.connect(self.toggle_orientation)
        self.layout.addWidget(self.toggle_button)

        self.central_widget.setLayout(self.layout)

        # OpenCV bridge
        self.bridge = CvBridge()
        self.video_writer = None
        self.is_recording = False

        self.subscriber_thread = Thread(target=self.start_camera_subscriber)
        self.subscriber_thread.daemon = True
        self.subscriber_thread.start()

        self.imu_subscriber_thread = Thread(target=self.start_imu_subscriber)
        self.imu_subscriber_thread.daemon = True
        self.imu_subscriber_thread.start()

        self.gps_subscriber_thread = Thread(target=self.start_gps_subscriber)
        self.gps_subscriber_thread.daemon = True
        self.gps_subscriber_thread.start()

        self.lidar_subscriber_thread = Thread(target=self.start_lidar_subscriber)
        self.lidar_subscriber_thread.daemon = True
        self.lidar_subscriber_thread.start()

        self.orientation_representation = "quaternion"
    def start_camera_subscriber(self):
        rospy.Subscriber('/rrbot/camera1/image_raw', Image, self.image_callback)
        rospy.spin()

    def start_imu_subscriber(self):
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.spin()

    def start_gps_subscriber(self):
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        rospy.spin()

    def start_lidar_subscriber(self):
        rospy.Subscriber('/rrbot/laser/scan', LaserScan, self.lidar_callback)
        rospy.spin()


    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        self.video_label.setPixmap(pixmap)
        self.video_label.setScaledContents(True)

        # Write to video if recording
        if self.is_recording:
            if self.video_writer is None:
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                self.video_writer = cv2.VideoWriter('recorded_video.avi', fourcc, 20.0, (width, height))
            
            self.video_writer.write(cv_image)

    def imu_callback(self, msg):
        linear_acceleration = f"{msg.linear_acceleration.x:.2f}, {msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f}"
        angular_velocity = f"{msg.angular_velocity.x:.2f}, {msg.angular_velocity.y:.2f}, {msg.angular_velocity.z:.2f}"
        
        if self.orientation_representation == "quaternion":
            orientation = f"{msg.orientation.x:.2f}, {msg.orientation.y:.2f}, {msg.orientation.z:.2f}, {msg.orientation.w:.2f}"
        else:  # Assume Euler for the sake of example
            roll, pitch, yaw = quaternion_to_euler_client(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            orientation = f"roll: {roll:.2f}, pitch: {pitch:.2f}, yaw: {yaw:.2f}"
        self.linear_acceleration_label.setText(f"Linear Acceleration (x, y, z): {linear_acceleration}")
        self.angular_velocity_label.setText(f"Angular Velocity (x, y, z): {angular_velocity}")
        self.orientation_label.setText(f"Orientation: {orientation}")


    # Write to CSV if recording
        if self.is_recording:
            self.write_to_csv(linear_acceleration, angular_velocity, orientation)


    def toggle_orientation(self):
        if self.orientation_representation == "quaternion":
            self.orientation_representation = "euler"
        else:
            self.orientation_representation = "quaternion"

    def gps_callback(self, msg):
        lat = f"lat: {msg.latitude:.6f}"
        long = f"long: {msg.longitude:.6f}"
        alt = f"alt: {msg.altitude:.2f} m"
        gps_data = f"{lat}\n{long}\n{alt}"
        self.gps_label.setText(gps_data)

    def lidar_callback(self, msg):
        if self.detect_obstacle(msg):
            self.obstacle_label.setText("Obstacle Detected!")
        else:
            self.obstacle_label.setText("Path Clear")

    def detect_obstacle(self, laser_scan):
        threshold_distance = 3.5
        for i in range(len(laser_scan.ranges)):
            if laser_scan.ranges[i] < threshold_distance:
                return True
        return False

    def capture_screenshot(self):
        pixmap = self.video_label.pixmap()
        if pixmap:
            dir_path = "/home/aditi/Pictures/Webcam"
            file_path = f"{dir_path}/screenshot.png"
            pixmap.save(file_path)

    def toggle_record(self):
        if not self.is_recording:
            self.is_recording = True
            self.record_button.setText('Stop Recording')
        else:
            self.is_recording = False
            self.record_button.setText('Record')
            if self.video_writer:
                self.video_writer.release()
                self.video_writer = None
    def write_to_csv(self, linear_acceleration, angular_velocity, orientation):
        if self.csv_writer:
            self.csv_writer.writerow([linear_acceleration, angular_velocity, orientation])

    def csv_record(self):
        if not self.is_recording:
            self.is_recording = True
            self.csv_button.setText('Stop Recording')
        
            # Open CSV file for writing
            self.csv_file = open('imu_data.csv', 'w')
            fieldnames = ['Linear Acceleration', 'Angular Velocity', 'Orientation']
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(fieldnames)
        else:
            self.is_recording = False
            self.csv_button.setText('Record')
        
            # Close CSV file
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
                self.csv_writer = None


if __name__ == '__main__':
    rospy.init_node('rover_display', anonymous=True)
    app = QApplication(sys.argv)
    window = RoverDisplay()
    window.show()
    sys.exit(app.exec_())