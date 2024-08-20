#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

class GroundRemovalNode : public rclcpp::Node
{
public:
  GroundRemovalNode()
  : Node("ground_removal_node")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/zed2_left_camera/points", 10, std::bind(&GroundRemovalNode::pointCloudCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_removed_points", 10);
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PCLPointCloud2* pcl_pc = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*msg, *pcl_pc);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pcl_pc, *cloud);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.08);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
      RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
      return;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toPCLPointCloud2(*cloud_filtered, *pcl_pc);
    pcl_conversions::fromPCL(*pcl_pc, output);
    output.header.frame_id = msg->header.frame_id;
    pub_->publish(output);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundRemovalNode>());
  rclcpp::shutdown();
  return 0;
}
