#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

class GPS {
public:
    double latitude;
    double longitude;
    GPS(double lat, double lon) {
    latitude = lat;
    longitude = lon;
}

};

class Rover {
public:
    double latitude;
    double longitude;
    double orientation; 
    Rover() {
    latitude = 0.0;
    longitude = 0.0;
    orientation = 0.0;
}

   

    void setInitialPosition(const GPS& initialPose) {
        latitude = initialPose.latitude;
        longitude = initialPose.longitude;
    }

    void moveTo(const GPS& target) {
        double distance = calculateDistance(latitude, longitude, target.latitude, target.longitude);
        ROS_INFO("Moving to target. Distance: %f meters.", distance);

        //  angle to the target
        orientation = atan2(target.longitude - longitude, target.latitude - latitude);

        latitude = target.latitude;
        longitude = target.longitude;

        ROS_INFO("Arrived at target. Current position: %f, %f", latitude, longitude);
    }

    double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
        double earthRadius = 6371000.0;
        double dLat = degreesToRadians(lat2 - lat1);
        double dLon = degreesToRadians(lon2 - lon1);
        double a = sin(dLat / 2) * sin(dLat / 2) + cos(degreesToRadians(lat1)) * cos(degreesToRadians(lat2)) *
                                                  sin(dLon / 2) * sin(dLon / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        return earthRadius * c;
    }

    double degreesToRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }
};

bool reachedGoal = false;

void targetGpsCallback(const sensor_msgs::NavSatFix::ConstPtr& targetGpsMsg, Rover& myRover, geometry_msgs::Twist& cmd_vel, const GPS& destination) {
    GPS targetGPS(targetGpsMsg->latitude, targetGpsMsg->longitude);
    if (!reachedGoal) {
        myRover.moveTo(targetGPS);

        //  distance to the target
        double distanceToTarget = myRover.calculateDistance(myRover.latitude, myRover.longitude, destination.latitude, destination.longitude);

        
        double thresholdDistance = 0.7; 

        
        if (distanceToTarget < thresholdDistance) {
            ROS_INFO("Rover has reached the goal!");
            reachedGoal = true;
            
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
        } else {
            
            double angleToTarget = atan2(destination.longitude - myRover.longitude, destination.latitude - myRover.latitude);

            //  difference in orientation
            double angleDifference = angleToTarget - myRover.orientation;

            
            ROS_INFO("Angle to target: %f", angleToTarget);
            ROS_INFO("Current orientation: %f", myRover.orientation);
            ROS_INFO("Angle difference: %f", angleDifference);

            
            cmd_vel.linear.x = 1.0;
            cmd_vel.angular.z = 2.0 * angleDifference;  

            // Update the rover's orientation
            myRover.orientation = angleToTarget;
        }
    }
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "gps");
    ros::NodeHandle nh;

    //  publisher f
    ros::Publisher roverPositionPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    
    GPS destination(0.0,0.0);  

    // subscriber 
    Rover myRover;
    geometry_msgs::Twist cmd_vel;
    ros::Subscriber targetGpsSub = nh.subscribe<sensor_msgs::NavSatFix>(
        "/gps/fix",
        1,
        [&myRover, &cmd_vel, &destination](const sensor_msgs::NavSatFix::ConstPtr& targetGpsMsg) {
            targetGpsCallback(targetGpsMsg, myRover, cmd_vel, destination);
        }
    );

    // Main loop
    ros::Rate loop_rate(10);  
    
    while (ros::ok()) {
        
        roverPositionPub.publish(cmd_vel);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
