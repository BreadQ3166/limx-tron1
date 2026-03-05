#include <cstdio>
#include <memory>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lidar_odometry/lidar_odometry.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include "tf2_eigen/tf2_eigen.hpp"

class LidarOdometryNode : public rclcpp::Node
{
  public:
    LidarOdometryNode() : Node("lidar_odometry_node")
    {
      RCLCPP_INFO(this->get_logger(), "lidar_odometry_node");

      parameter_initilization();

      double max_correspondence_distance;
      double transformation_epsilon;
      double maximum_iterations;
      std::string scan_topic_name;
      std::string odom_topic_name;

      this->get_parameter("max_correspondence_distance", max_correspondence_distance);
      this->get_parameter("transformation_epsilon", transformation_epsilon);
      this->get_parameter("maximum_iterations", maximum_iterations);
      this->get_parameter("scan_topic_name", scan_topic_name);
      this->get_parameter("odom_topic_name", odom_topic_name);

      RCLCPP_INFO(this->get_logger(), "===== Configuration =====");

      RCLCPP_INFO(this->get_logger(), "max_correspondence_distance: %.4f", max_correspondence_distance);
      RCLCPP_INFO(this->get_logger(), "transformation_epsilon: %.4f", transformation_epsilon);
      RCLCPP_INFO(this->get_logger(), "maximum_iterations %.4f", maximum_iterations);
      RCLCPP_INFO(this->get_logger(), "scan_topic_name: %s", scan_topic_name.c_str());
      RCLCPP_INFO(this->get_logger(), "odom_topic_name: %s", odom_topic_name.c_str());

      lidar_odometry_ptr = std::make_shared<LidarOdometry>(max_correspondence_distance, transformation_epsilon, maximum_iterations);

      odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name, 100);
      scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_name, 1000, std::bind(&LidarOdometryNode::scan_callback, this, std::placeholders::_1)
      );

      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    private:
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
      rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // 【新增】
      std::shared_ptr<LidarOdometry> lidar_odometry_ptr;

      void parameter_initilization() {
        this->declare_parameter<double>("max_correspondence_distance", 1.0);
        this->declare_parameter<double>("transformation_epsilon", 0.005);
        this->declare_parameter<double>("maximum_iterations", 30);

        // this->declare_parameter<std::string>("odom_topic_name", "scan_odom");

        this->declare_parameter<std::string>("scan_topic_name", "scan");
        this->declare_parameter<std::string>("odom_topic_name", "scan_odom");
      }

      void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        auto point_cloud_msg = laser2cloudmsg(scan_msg);
        auto pcl_point_cloud = cloudmsg2cloud(point_cloud_msg);

        auto scan_data = std::make_shared<ScanData>();
        scan_data->timestamp = scan_msg->header.stamp.sec + scan_msg->header.stamp.nanosec / 1e9;
        scan_data->point_cloud = pcl_point_cloud;

        lidar_odometry_ptr->process_scan_data(scan_data);
        publish_odometry(scan_msg->header.stamp);
      }

      void publish_odometry(const rclcpp::Time & stamp) {
        auto state = lidar_odometry_ptr->get_state();
        std::string fixed_id = "scan_odom";
        std::string child_id = "base_Link"; 
        auto now = stamp;

        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.frame_id = fixed_id;
        odom_msg.child_frame_id = child_id;
        odom_msg.header.stamp = now;

        odom_msg.pose.pose = Eigen::toMsg(state->pose);
        odom_msg.twist.twist = Eigen::toMsg(state->velocity);

        odom_publisher->publish(odom_msg);

        // 2. 发布 TF 变换 (新增部分)
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = fixed_id;
        t.child_frame_id = child_id;

        // 将 Eigen 变换转换为 TF 变换
        // 注意：这里需要根据你的 Eigen 数据结构进行转换
        // 如果 state->pose 是 Eigen::Isometry3d
        t.transform.translation.x = state->pose.translation().x();
        t.transform.translation.y = state->pose.translation().y();
        t.transform.translation.z = state->pose.translation().z();
        
        Eigen::Quaterniond q(state->pose.rotation());
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // 发送变换
        tf_broadcaster_->sendTransform(t);
      }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
