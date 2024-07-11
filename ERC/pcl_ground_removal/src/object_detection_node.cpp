#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

class ObjectDetectionNode : public rclcpp::Node
{
public:
  ObjectDetectionNode()
  : Node("object_detection_node")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/zed2_left_camera/points", 10, std::bind(&ObjectDetectionNode::pointCloudCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/objects_detected", 10);
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PCLPointCloud2* pcl_pc = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*msg, *pcl_pc);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pcl_pc, *cloud);

    // Perform segmentation to detect slopes and craters (objects)
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    segmentObjects(cloud, inliers);

    // Extract the points representing objects
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);  // Extract the indices that are true (objects)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*cloud_filtered);

    // Publish the filtered point cloud
    sensor_msgs::msg::PointCloud2 output;
    pcl::toPCLPointCloud2(*cloud_filtered, *pcl_pc);
    pcl_conversions::fromPCL(*pcl_pc, output);
    output.header = msg->header;
    pub_->publish(output);
  }

 void segmentObjects(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointIndices::Ptr& inliers)
{
    // Setup SAC segmentation parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);  // Assuming ground plane is dominant
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);  // Adjust as needed based on your data

    // Set input cloud and segment to get indices of objects (slopes and craters)
    seg.setInputCloud(cloud);
    
    // Ensure coefficients_ is initialized properly before using it
    if (!coefficients_)
        coefficients_ = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);

    seg.segment(*inliers, *coefficients_);  // Use dereferenced pointer here

    if (inliers->indices.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Could not find any objects (slopes and craters) in the point cloud.");
        return;
    }
}


  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  pcl::ModelCoefficients::Ptr coefficients_;  // Store coefficients for debugging or further analysis
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
