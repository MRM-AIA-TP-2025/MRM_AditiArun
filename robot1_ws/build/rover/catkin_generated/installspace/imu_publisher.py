import rospy
from sensor_msgs.msg import Imu
import json
import socket
import math

HOST = '192.168.229.198'  # Your laptop's IP address
PORT = 12346  # Port number

def euler_to_quaternion(roll, pitch, yaw):
   
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return x, y, z, w

def imu_publisher():
    rospy.init_node('imu_publisher', anonymous=True)
    imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        rospy.loginfo(f"Listening on {HOST}:{PORT}")
        conn, addr = s.accept()
        rospy.loginfo(f"Connected by {addr}")
        
        while not rospy.is_shutdown():
            data = conn.recv(1024)
            if not data:
                break
            rospy.loginfo(f"Received data: {data.decode()}")  
            
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "map"
            
            Values = data.decode().strip().split(',')
            values = []
            for i in Values:
                i = i[0:8]
                values.append(i)
            rospy.loginfo(f"Received data: {values}")  

            if len(values) == 9: 
                try:
                    imu_msg.linear_acceleration.x = float(values[0])
                    imu_msg.linear_acceleration.y = float(values[1])
                    imu_msg.linear_acceleration.z = float(values[2])

                    imu_msg.angular_velocity.x = float(values[3])
                    imu_msg.angular_velocity.y = float(values[4])
                    imu_msg.angular_velocity.z = float(values[5])

                    roll = float(values[8])
                    pitch = float(values[7])
                    yaw = float(values[6])

                    # Convert Euler angles to quaternions
                    x, y, z, w = euler_to_quaternion(roll, pitch, yaw)

                    imu_msg.orientation.x = x
                    imu_msg.orientation.y = y
                    imu_msg.orientation.z = z
                    imu_msg.orientation.w = w
                    
                    imu_pub.publish(imu_msg)
                except ValueError as e:
                    rospy.logwarn(f"Error parsing data: {e}")
            rate.sleep()

if __name__ == "__main__":
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass
