
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <map>
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
        return degrees * M_PI / 180.0;}

    bool detectObstacle(const sensor_msgs::LaserScan::ConstPtr& laserScan) {
        double thresholdDistance = 1.8; // Define a threshold distance for obstacle detection
        for (size_t i = 0; i < laserScan->ranges.size(); ++i) {
            if (laserScan->ranges[i] < thresholdDistance) {
                ROS_WARN("Obstacle detected at angle %f with distance %f", laserScan->angle_min + i * laserScan->angle_increment, laserScan->ranges[i]);
                return true; // Obstacle detected
            }
        }
        
        return false; // No obstacle detected
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
            ROS_INFO("AAADistance: %f meters.", distanceToTarget);
            if (distanceToTarget<2)
            {
                cmd_vel.linear.x = 0.25*distanceToTarget;
                cmd_vel.angular.z = 0.5* angleDifference;  

            }
            cmd_vel.linear.x = 1.0;
            cmd_vel.angular.z = 1.5 * angleDifference;  

            // Update the rover's orientation
            myRover.orientation = angleToTarget;
        }
    }
}
void take_action(const std::map<std::string, double>& regions, geometry_msgs::Twist& cmd_vel) {
    double threshold_dist = 1.5;
    double linear_speed = 0.7;
    double angular_speed = 1.2;

    std::string state_description = "";

    if (regions.at("front") < threshold_dist && regions.at("left") < threshold_dist && regions.at("right") < threshold_dist) {
        state_description = "case 7 - front and left and right";
        cmd_vel.linear.x = -linear_speed;
        cmd_vel.angular.z = angular_speed;
    } else if (regions.at("front") < threshold_dist && regions.at("left") > threshold_dist && regions.at("right") > threshold_dist) {
        state_description = "case 2 - front";
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = angular_speed;
    } else if (regions.at("front") > threshold_dist && regions.at("left") > threshold_dist && regions.at("right") < threshold_dist) {
        state_description = "case 3 - right";
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = -angular_speed;
    } else if (regions.at("front") > threshold_dist && regions.at("left") < threshold_dist && regions.at("right") > threshold_dist) {
        state_description = "case 4 - left";
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = angular_speed;
    } else if (regions.at("front") < threshold_dist && regions.at("left") > threshold_dist && regions.at("right") < threshold_dist) {
        state_description = "case 5 - front and right";
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = -angular_speed;
    } else if (regions.at("front") < threshold_dist && regions.at("left") < threshold_dist && regions.at("right") > threshold_dist) {
        state_description = "case 6 - front and left";
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = angular_speed;
    } else if (regions.at("front") > threshold_dist && regions.at("left") < threshold_dist && regions.at("right") < threshold_dist) {
        state_description = "case 8 - obstacle on the left and right";
        cmd_vel.linear.x = -linear_speed;
        cmd_vel.angular.z = angular_speed;
    } else {
        state_description = "unknown case";
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = -angular_speed;
    }

    ROS_INFO("%s", state_description.c_str());
}



/*void take_action(const std::map<std::string, double>& regions, geometry_msgs::Twist& cmd_vel) {
    double threshold_dist = 1.0;
    double linear_speed = 0.6;
    double angular_speed = 1;

    std::string state_description = "";

    
    if (regions.at("front") < threshold_dist && regions.at("left") < threshold_dist && regions.at("right") < threshold_dist) {
        state_description = "case 7 - front and left and right";
        cmd_vel.linear.x = -linear_speed;
        cmd_vel.angular.z = angular_speed;
    } else if (regions.at("front") < threshold_dist && regions.at("left") > threshold_dist && regions.at("right") > threshold_dist) {
        state_description = "case 2 - front";
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = angular_speed;
    } else if (regions.at("front") > threshold_dist && regions.at("left") > threshold_dist && regions.at("right") < threshold_dist) {
        state_description = "case 3 - right";
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = -angular_speed;
    } else if (regions.at("front") > threshold_dist && regions.at("left") < threshold_dist && regions.at("right") > threshold_dist) {
        state_description = "case 4 - left";
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = angular_speed;
    } else if (regions.at("front") < threshold_dist && regions.at("left") > threshold_dist && regions.at("right") < threshold_dist) {
        state_description = "case 5 - front and right";
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = -angular_speed;
    } else if (regions.at("front") < threshold_dist && regions.at("left") < threshold_dist && regions.at("right") > threshold_dist) {
        state_description = "case 6 - front and left";
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = angular_speed;
    } else if (regions.at("front") > threshold_dist && regions.at("left") < threshold_dist && regions.at("right") < threshold_dist) {
        state_description = "case 8 - obstacle on the left and right";
        cmd_vel.linear.x = -linear_speed;
        cmd_vel.angular.z = angular_speed;
    } else {
        state_description = "unknown case";
    }

    ROS_INFO("%s AAAAA", state_description.c_str());
}*/

/*oid take_action(const std::map<std::string, double>& regions, ros::Publisher& pub) {
    double threshold_dist = 1.0;
    double linear_speed = 0.6;
    double angular_speed = 1;

    geometry_msgs::Twist msg;
    double linear_x = 0;
    double angular_z = 0;
    std::string state_description = "";

    if (regions.at("front") > threshold_dist && regions.at("left") > threshold_dist && regions.at("right") > threshold_dist) {
        state_description = "case 1 - no obstacle";
        linear_x = linear_speed;
        angular_z = 0;
    } else if (regions.at("front") < threshold_dist && regions.at("left") < threshold_dist && regions.at("right") < threshold_dist) {
        state_description = "case 7 - front and left and right";
        linear_x = -linear_speed;
        angular_z = angular_speed;
    } else if (regions.at("front") < threshold_dist && regions.at("left") > threshold_dist && regions.at("right") > threshold_dist) {
        state_description = "case 2 - front";
        linear_x = 0;
        angular_z = angular_speed;
    } else if (regions.at("front") > threshold_dist && regions.at("left") > threshold_dist && regions.at("right") < threshold_dist) {
        state_description = "case 3 - right";
        linear_x = 0;
        angular_z = -angular_speed;
    } else if (regions.at("front") > threshold_dist && regions.at("left") < threshold_dist && regions.at("right") > threshold_dist) {
        state_description = "case 4 - left";
        linear_x = 0;
        angular_z = angular_speed;
    } else if (regions.at("front") < threshold_dist && regions.at("left") > threshold_dist && regions.at("right") < threshold_dist) {
        state_description = "case 5 - front and right";
        linear_x = 0;
        angular_z = -angular_speed;
    } else if (regions.at("front") < threshold_dist && regions.at("left") < threshold_dist && regions.at("right") > threshold_dist) {
        state_description = "case 6 - front and left";
        linear_x = 0;
        angular_z = angular_speed;
    } else if (regions.at("front") > threshold_dist && regions.at("left") < threshold_dist && regions.at("right") < threshold_dist) {
        state_description = "case 8 - obstacle on the left and right";
        linear_x = -linear_speed;
        angular_z = angular_speed;
    } else {
        state_description = "unknown case";
    }

    ROS_INFO("%s AAAAA", state_description.c_str());

    msg.linear.x = linear_x;
    msg.angular.z = angular_z;

    // Publish the twist message
    pub.publish(msg);
}*/
/*void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan, Rover& myRover, geometry_msgs::Twist& cmd_vel, ros::Publisher& pub) {
    if (!reachedGoal) {
        std::map<std::string, double> regions;
        
        regions["right"] = std::min(std::min(static_cast<double>(laserScan->ranges[0]), static_cast<double>(laserScan->ranges[1])), 10.0);
        regions["front"] = std::min(std::min(static_cast<double>(laserScan->ranges[89]), static_cast<double>(laserScan->ranges[90])), 10.0);
        regions["left"] = std::min(std::min(static_cast<double>(laserScan->ranges[178]), static_cast<double>(laserScan->ranges[179])), 10.0);

    
        if (myRover.detectObstacle(laserScan)) {
            take_action(regions, pub);
        }
    }
}*/
/*void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan, Rover& myRover, geometry_msgs::Twist& cmd_vel, ros::Publisher& pub) {
    if (!reachedGoal) {
        std::map<std::string, double> regions;
        regions["right"] = std::min(std::min(static_cast<double>(laserScan->ranges[0]), static_cast<double>(laserScan->ranges[1])), 10.0);
        regions["front"] = std::min(std::min(static_cast<double>(laserScan->ranges[2]), static_cast<double>(laserScan->ranges[3])), 10.0);
        regions["left"] = std::min(std::min(static_cast<double>(laserScan->ranges[4]), static_cast<double>(laserScan->ranges[5])), 10.0);
    
        if (myRover.detectObstacle(laserScan)) {
            take_action(regions, cmd_vel); // Pass cmd_vel instead of pub
        }
    }
}*/

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan, Rover& myRover, geometry_msgs::Twist& cmd_vel, ros::Publisher& pub) {
    if (!reachedGoal) {
        std::map<std::string, double> regions;
        regions["right"] = std::min(std::min(static_cast<double>(laserScan->ranges[0]), static_cast<double>(laserScan->ranges[1])), 10.0);
        regions["front"] = std::min(std::min(static_cast<double>(laserScan->ranges[2]), static_cast<double>(laserScan->ranges[3])), 10.0);
        regions["left"] = std::min(std::min(static_cast<double>(laserScan->ranges[4]), static_cast<double>(laserScan->ranges[5])), 10.0);
    
        if (myRover.detectObstacle(laserScan)) {
            take_action(regions, cmd_vel); // Pass cmd_vel instead of pub
            pub.publish(cmd_vel); // Publish the twist message
        }
    }
}


int main(int argc, char **argv) {
    
    ros::init(argc, argv, "obstacle");
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
    
    ros::Subscriber lidarSub = nh.subscribe<sensor_msgs::LaserScan>(
    "/rrbot/laser/scan",
    1,
    [&myRover, &cmd_vel, &roverPositionPub](const sensor_msgs::LaserScan::ConstPtr& laserScan) {
        lidarCallback(laserScan, myRover, cmd_vel, roverPositionPub);
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