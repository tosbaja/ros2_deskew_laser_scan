#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/executor.hpp"
#include "rcl/node.h"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/buffer_core.h>
#include <iostream>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using std::placeholders::_1;
typedef pcl::PointXYZI PointType;
class DeskewLaserScan : public rclcpp::Node
{
public:
  DeskewLaserScan(const rclcpp::NodeOptions &options);

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_1;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_2;
  
  rclcpp::CallbackGroup::SharedPtr callback_group_scan;
  rclcpp::CallbackGroup::SharedPtr callback_group_odom;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription2_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr subscription3_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2::Transform laser_position_tf;
  pcl::PointCloud<PointType>::Ptr lidar_point_cloud_cartesian;

  void odom_callback(const nav_msgs::msg::Odometry &msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg);
  void pointcloud_callback(const sensor_msgs::msg::PointCloud::ConstSharedPtr &pointcloud_msg);

  void publish_pcl_data(const pcl::PointCloud<PointType>::Ptr &this_cloud, const rclcpp::Time &this_stamp, const std::string &this_frame);
  const tf2::Vector3 deskew_point(const tf2::Transform &lazer_pose_change,const tf2::Vector3 &relevant_lazer_point);
  const tf2::Transform move_robot_pose(const tf2::Transform &current_robot_pose,const  float &linear_velocity,const float &angular_velocity,const float &duration);
  const tf2::Vector3 perform_whole_deskew(const float &passed_time, const float &x_cord,const float &y_cord);
  template <typename ParamType> void declare_and_get_param(const std::string &param_name, const ParamType &param_val, ParamType &param_var);

  //params
  std::string subscribe_laser_topic;
  std::string subscribe_odometry_topic;
  std::string subscribe_pointcloud_topic;
  std::string deskewed_pointcloud2_publish_topic;
  std::string deskewed_scan_publish_topic;      
  std::string base_frame;
  std::string laser_frame;
  float current_speed_rot;
  float current_speed_lin;
  int deskewed_scan_fixed_size;
  bool pub_pc2;
  bool use_deskew_from_scan;
  bool got_transforms;
  bool invalid_range_is_inf;
  float scan_start_angle;
  float min_angle;
  float max_angle;
  float min_range;
  float max_range;
};
