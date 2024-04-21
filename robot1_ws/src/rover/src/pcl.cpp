#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <map>
#include <cmath>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub;
//geometry_msgs::Twist cmd_vel; // Declaring cmd_vel as a global variable



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

        // Angle to the target
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

void take_action(const pcl::PointXYZRGB& centroid, double robot_orientation, double linear_speed, geometry_msgs::Twist& cmd_vel)  {
    double angular_speed = 1.85;
    double wall_following_distance = 1.5; // Distance to maintain from the wall

    // Calculate the distance from the rover to the centroid
    double distance_to_centroid = sqrt(centroid.x * centroid.x + centroid.y * centroid.y);

    // Analyze position and take action accordingly
    if (distance_to_centroid < wall_following_distance) {
        // Obstacle detected within the threshold distance, take action
        if (centroid.x > 0 && centroid.y > 0) {
            // Move left
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = angular_speed*1.2;
            pub.publish(cmd_vel);
            
        } else if (centroid.x > 0 && centroid.y < 0) {
            // Move left
            cmd_vel.linear.x = linear_speed;
            cmd_vel.angular.z = 0.0;
            pub.publish(cmd_vel);
            
        } else if (centroid.x < 0 && centroid.y > 0) {
            // Move right
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = -angular_speed*1.2;
            pub.publish(cmd_vel);
            
        } else if (centroid.x < 0 && centroid.y < 0) {
            // Move right
            cmd_vel.linear.x = linear_speed;
            cmd_vel.angular.z = 0;
            pub.publish(cmd_vel);
           
        }
    } else {
        // Obstacle detected, but not within the threshold distance, no action needed
        ROS_INFO("Obstacle detected, but not within the threshold distance.");
        cmd_vel.linear.x = linear_speed;
        cmd_vel.angular.z = 0.6;
        pub.publish(cmd_vel);
        
    }
}


void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, Rover& myRover,geometry_msgs::Twist& cmd_vel) {
    // Convert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZRGB>
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Preprocessing: Filter out points outside a specified range
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 20.0); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pass.filter(*filtered_cloud);

    // Ground plane elimination using RANSAC
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);//coeffs of plane
    pcl::PointIndices::Ptr inlier_indices(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setInputCloud(filtered_cloud);
    seg.setModelType(pcl::SACMODEL_PLANE);//model type to identify the planes, looks for planar surfaces
    //RANSAC is an iterative method used for robust estimation of model parameters from a set of observed data points, particularly in the presence of outliers.
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05); //dist for point to be inliner
    seg.segment(*inlier_indices, *coefficients);

    // Extract objects (non-ground points) from the filtered cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(filtered_cloud);
    extract.setIndices(inlier_indices);
    extract.setNegative(true);//all inliners extracted
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    extract.filter(*segmented_cloud);

    // Cluster extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.04);
    ec.setMinClusterSize(200);   
    ec.setMaxClusterSize(50000);  
    ec.setInputCloud(segmented_cloud);
    ec.extract(cluster_indices);

    // Check if an obstacle is detected
    if (cluster_indices.size() > 0) {
        // Determine position of the first cluster (assuming only one is detected)
        pcl::PointXYZRGB centroid;
        size_t total_points = 0;
        float centroid_x = 0.0;
        float centroid_y = 0.0;
        float centroid_z = 0.0;

        for (const auto& cluster : cluster_indices) {
            for (const auto& index : cluster.indices) {
                centroid_x += segmented_cloud->points[index].x;
                centroid_y += segmented_cloud->points[index].y;
                centroid_z += segmented_cloud->points[index].z;
                total_points++;
            }
        }

        centroid.x = centroid_x / total_points;
        centroid.y = centroid_y / total_points;
        centroid.z = centroid_z / total_points - centroid_x;

        // Calculate the distance from the rover to the centroid
        double distance_to_centroid = sqrt(centroid.x * centroid.x + centroid.y * centroid.y);

        // Pass the linear speed to the take_action function
        double linear_speed = 0.3; 
        double robot_orientation = myRover.orientation;
        take_action(centroid, robot_orientation, linear_speed,cmd_vel);
    } else {
        // No obstacle detected
        ROS_INFO("No obstacle detected.");
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pcl");
    ros::NodeHandle nh;

    // Assign the publisher to the global variable pub
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Destination GPS coordinates
    GPS destination(0.0, 0.0);

    // Subscriber for point cloud data
    Rover myRover;
    geometry_msgs::Twist cmd_vel;
    ros::Subscriber targetGpsSub = nh.subscribe<sensor_msgs::NavSatFix>(
        "/gps/fix",
        1,
        [&myRover, &cmd_vel, &destination](const sensor_msgs::NavSatFix::ConstPtr& targetGpsMsg) {
            targetGpsCallback(targetGpsMsg, myRover, cmd_vel, destination);
        }
    );
    ros::Subscriber cloudSub = nh.subscribe<sensor_msgs::PointCloud2>(
        "/zed2_cam/depth/points",
        1,
        [&myRover,&cmd_vel](const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
            cloudCallback(cloud_msg, myRover,cmd_vel);
        }
    );

    // Main loop
    ros::Rate loop_rate(10);  
    while (ros::ok()) {
        // Publish velocity command
        pub.publish(cmd_vel);

        // Wait for callbacks and control loop rate
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
