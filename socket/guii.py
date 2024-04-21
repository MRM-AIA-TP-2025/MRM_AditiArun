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

        self.layout.addWidget(self.video_label, 0, 0, 5, 1)
        self.layout.addWidget(self.imu_label, 0, 1)
        self.layout.addWidget(self.gps_label, 1, 1)
        self.layout.addWidget(self.obstacle_label, 2, 1)
        self.layout.addWidget(self.screenshot_button, 3, 1)
        self.layout.addWidget(self.record_button, 4, 1)

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
        linear_acceleration = f"Linear Acceleration (x, y, z): {msg.linear_acceleration.x:.2f}, {msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f}"
        angular_velocity = f"Angular Velocity (x, y, z): {msg.angular_velocity.x:.2f}, {msg.angular_velocity.y:.2f}, {msg.angular_velocity.z:.2f}"
        orientation = f"Orientation (x, y, z, w): {msg.orientation.x:.2f}, {msg.orientation.y:.2f}, {msg.orientation.z:.2f}, {msg.orientation.w:.2f}"
        imu_data = f"{linear_acceleration}\n{angular_velocity}\n{orientation}"
        self.imu_label.setText(imu_data)

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

if __name__ == '__main__':
    rospy.init_node('rover_display', anonymous=True)
    app = QApplication(sys.argv)
    window = RoverDisplay()
    window.show()
    sys.exit(app.exec_())
