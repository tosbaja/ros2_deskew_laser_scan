#include "ros2_deskew_laser_scan/ros2_deskew_laser_scan.h"

using namespace std::chrono_literals;

DeskewLaserScan::DeskewLaserScan(const rclcpp::NodeOptions &options)
    : Node("ros2_deskew_laser_scan", options), current_speed_rot{0.0}, current_speed_lin{0.0}, got_transforms{false}
{
  callback_group_odom = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_scan = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  auto odom_sub_opts = rclcpp::SubscriptionOptions();
  odom_sub_opts.callback_group = callback_group_odom;
  auto scan_sub_opts = rclcpp::SubscriptionOptions();
  scan_sub_opts.callback_group = callback_group_scan;

  declare_and_get_param<int>("deskewed_scan_fixed_size", 1200, deskewed_scan_fixed_size);
  declare_and_get_param<std::string>("subscribe_odometry_topic", "/diff_cont/odom", subscribe_odometry_topic);
  declare_and_get_param<std::string>("base_frame", "base_link", base_frame);
  declare_and_get_param<std::string>("laser_frame", "laser_frame", laser_frame);
  declare_and_get_param<bool>("publish_pointcoud2", false, pub_pc2);
  declare_and_get_param<bool>("invalid_range_is_inf", true, invalid_range_is_inf);

  declare_and_get_param<std::string>("deskew_from_scan.subscribe_laser_topic", "/scan_with_start_index", subscribe_laser_topic);
  declare_and_get_param<bool>("deskew_from_scan.use_deskew_from_scan", false, use_deskew_from_scan);
  declare_and_get_param<float>("deskew_from_scan.scan_start_angle", 0.35, scan_start_angle);

  declare_and_get_param<float>("deskew_from_pointcloud.min_angle", -3.1415927410125732, min_angle);
  declare_and_get_param<float>("deskew_from_pointcloud.max_angle", -3.1415927410125732, max_angle);
  declare_and_get_param<float>("deskew_from_pointcloud.min_range", 0.05, min_range);
  declare_and_get_param<float>("deskew_from_pointcloud.max_range", 50.0, max_range);
  declare_and_get_param<std::string>("deskew_from_pointcloud.subscribe_pointcloud_topic", "/point_cloud", subscribe_pointcloud_topic);

  declare_and_get_param<std::string>("deskewed_scan_publish_topic", "/scan_deskewed", deskewed_scan_publish_topic);
  declare_and_get_param<std::string>("deskewed_pointcloud2_publish_topic", "/pointcloud2_deskewed", deskewed_pointcloud2_publish_topic);
  tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      subscribe_odometry_topic, rclcpp::SensorDataQoS(), std::bind(&DeskewLaserScan::odom_callback, this, _1), odom_sub_opts);
  if (use_deskew_from_scan)
    subscription2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        subscribe_laser_topic, rclcpp::SensorDataQoS(), std::bind(&DeskewLaserScan::scan_callback, this, _1), scan_sub_opts);
  if (!use_deskew_from_scan)
  subscription3_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
      subscribe_pointcloud_topic, rclcpp::SensorDataQoS(), std::bind(&DeskewLaserScan::pointcloud_callback, this, _1), scan_sub_opts);
  if (pub_pc2)
    pub_1 = this->create_publisher<sensor_msgs::msg::PointCloud2>(deskewed_pointcloud2_publish_topic, rclcpp::SensorDataQoS());
  pub_2 = create_publisher<sensor_msgs::msg::LaserScan>(deskewed_scan_publish_topic, rclcpp::SensorDataQoS());
}
template <typename ParamType>
void DeskewLaserScan::declare_and_get_param(const std::string &param_name, const ParamType &param_val, ParamType &param_var)
{
  declare_parameter(param_name, param_val);
  get_parameter(param_name, param_var);
  std::cout << param_name << ": " << param_var << std::endl;
  return;
}
/*
  TO-DO
    queue the messages and find in the queued messages (according to their timestamp)
    which are in the interval of the laser scan which will be deskewed. Interpolate the
    odom observations for a better deskew and use accordingly.
*/
void DeskewLaserScan::odom_callback(const nav_msgs::msg::Odometry &msg)
{
  if (!got_transforms)
  {
    try
    {
      geometry_msgs::msg::TransformStamped laser_position = tf_buffer_->lookupTransform(
          base_frame, laser_frame,
          tf2::TimePointZero, 100ms);
      tf2::fromMsg(laser_position.transform, laser_position_tf);
      got_transforms = true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Error occured while trying to get transform from base_link to laser frame");
      exit(0);
    }
  }
  current_speed_rot = msg.twist.twist.angular.z;
  current_speed_lin = msg.twist.twist.linear.x;
}
void DeskewLaserScan::pointcloud_callback(const sensor_msgs::msg::PointCloud::ConstSharedPtr &pointcloud_msg)
{
  if (!got_transforms)
    return;
  auto scan_msg_deskewed = std::make_shared<sensor_msgs::msg::LaserScan>();
  lidar_point_cloud_cartesian.reset(new pcl::PointCloud<PointType>());
  // fixed point size for deskewed scan_msg_in->laser_scan, and other information claculation
  double const_incre = (max_angle - min_angle) / (deskewed_scan_fixed_size - 1);

  // perepare deskewed scan
  scan_msg_deskewed->angle_max = max_angle;
  scan_msg_deskewed->angle_min = min_angle;
  scan_msg_deskewed->angle_increment = const_incre;
  scan_msg_deskewed->scan_time = 0.0;
  scan_msg_deskewed->time_increment = 0.0;
  scan_msg_deskewed->header = pointcloud_msg->header;
  scan_msg_deskewed->range_min = min_range;
  scan_msg_deskewed->range_max = max_range;
  scan_msg_deskewed->ranges.resize(deskewed_scan_fixed_size);
  scan_msg_deskewed->intensities.resize(deskewed_scan_fixed_size);
  if(invalid_range_is_inf)
  {
    fill(scan_msg_deskewed->ranges.begin(), scan_msg_deskewed->ranges.end(), INFINITY);
    fill(scan_msg_deskewed->intensities.begin(), scan_msg_deskewed->intensities.end(), INFINITY);
  }
  for (int i = 0; i < pointcloud_msg->points.size(); i++)
  {
    tf2::Vector3 point_transformed = perform_whole_deskew(pointcloud_msg->channels[1].values[i], pointcloud_msg->points[i].x, pointcloud_msg->points[i].y);
    const PointType point_transformed_3d{point_transformed.getX(),point_transformed.getY(),0.0};
    const auto deskewed_range = sqrt(pow(point_transformed_3d.x, 2) + pow(point_transformed_3d.y, 2));
    const auto deskewed_angle = atan2(point_transformed_3d.y, point_transformed_3d.x);
    const int index_skewed = std::ceil((deskewed_angle - min_angle) / const_incre);
    if(deskewed_range < min_range || deskewed_range > max_range) continue;
    if (index_skewed >= 0 && index_skewed < deskewed_scan_fixed_size)
    {
      scan_msg_deskewed->ranges[index_skewed] = deskewed_range;
      scan_msg_deskewed->intensities[index_skewed] = pointcloud_msg->channels[0].values[i];
    }
    lidar_point_cloud_cartesian->push_back(point_transformed_3d);
  }
  pub_2->publish(*scan_msg_deskewed);
  if (pub_pc2)
    publish_pcl_data(lidar_point_cloud_cartesian, pointcloud_msg->header.stamp, "laser_frame");
}
// for
void DeskewLaserScan::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg_in)
{
  if (!got_transforms)
    return;
  auto scan_msg_deskewed = std::make_shared<sensor_msgs::msg::LaserScan>();
  lidar_point_cloud_cartesian.reset(new pcl::PointCloud<PointType>());

  const int data_size_check = ((scan_msg_in->angle_max - scan_msg_in->angle_min) / scan_msg_in->angle_increment) + 1;

  const int real_scan_start = std::ceil((scan_start_angle - scan_msg_in->angle_min) / scan_msg_in->angle_increment);
  //std::cout << scan_msg_in->scan_start_index << "vs: " << real_scan_start << std::endl;

  // fixed point size for deskewed scan_msg_in->laser_scan, and other information claculation
  const double const_incre = (scan_msg_in->angle_max - scan_msg_in->angle_min) / (deskewed_scan_fixed_size - 1);

  // perepare deskewed scan
  scan_msg_deskewed->angle_max = scan_msg_in->angle_max;
  scan_msg_deskewed->angle_min = scan_msg_in->angle_min;
  scan_msg_deskewed->angle_increment = const_incre;
  scan_msg_deskewed->scan_time = 0.0;
  scan_msg_deskewed->time_increment = 0.0;
  scan_msg_deskewed->header = scan_msg_in->header;
  scan_msg_deskewed->range_min = scan_msg_in->range_min;
  scan_msg_deskewed->range_max = scan_msg_in->range_max;
  scan_msg_deskewed->ranges.resize(deskewed_scan_fixed_size);
  scan_msg_deskewed->intensities.resize(deskewed_scan_fixed_size);
  if(invalid_range_is_inf)
  {
    fill(scan_msg_deskewed->ranges.begin(), scan_msg_deskewed->ranges.end(), INFINITY);
    fill(scan_msg_deskewed->intensities.begin(), scan_msg_deskewed->intensities.end(), INFINITY);
  }
  if (data_size_check == scan_msg_in->ranges.size()) // check if msg is OK
  {
    int time_ticks;
    for (int i = 0; i < scan_msg_in->ranges.size(); i++)
    {
      // only do claculation if msg is valid
      if (scan_msg_in->ranges[i] >= scan_msg_in->range_min && scan_msg_in->ranges[i] <= scan_msg_in->range_max)
      {

        if (i <= real_scan_start)
          time_ticks = (real_scan_start - i);
        else
          time_ticks = (real_scan_start + scan_msg_in->ranges.size() - i);

        const float passed_time = scan_msg_in->time_increment * time_ticks;
        // transform to cartesian
        const float x_cord = scan_msg_in->ranges[i] * cos(scan_msg_in->angle_increment * i + scan_msg_in->angle_min);
        const float y_cord = scan_msg_in->ranges[i] * sin(scan_msg_in->angle_increment * i + scan_msg_in->angle_min);

        const tf2::Vector3 point_transformed = perform_whole_deskew(passed_time, x_cord, y_cord);

        const PointType point_transformed_3d{point_transformed.getX(),point_transformed.getY(),0.0};
        const float deskewed_range = sqrt(pow(point_transformed_3d.x, 2) + pow(point_transformed_3d.y, 2));
        const float deskewed_angle = atan2(point_transformed_3d.y, point_transformed_3d.x);
        const int index_skewed = std::ceil((deskewed_angle - scan_msg_in->angle_min) / const_incre);

        if (index_skewed >= 0 && index_skewed < deskewed_scan_fixed_size)
        {
          scan_msg_deskewed->ranges[index_skewed] = deskewed_range;
          scan_msg_deskewed->intensities[index_skewed] = scan_msg_in->intensities[i];
        }
        lidar_point_cloud_cartesian->push_back(point_transformed_3d);
      }
    }
    pub_2->publish(*scan_msg_deskewed);
    if (pub_pc2)
      publish_pcl_data(lidar_point_cloud_cartesian, scan_msg_in->header.stamp, "laser_frame");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "laser message corrupted, data size is incosistent with angle_increment pass!");
  }
}

const tf2::Vector3 DeskewLaserScan::perform_whole_deskew(const float &passed_time, const float &x_cord, const float &y_cord)
{
  tf2::Quaternion robot_rotation = tf2::Quaternion();
  robot_rotation.setRPY(0, 0, 0);
  tf2::Transform robot_pose = tf2::Transform(robot_rotation, tf2::Vector3(0, 0, 0));

  const tf2::Transform changed_robot_pose = move_robot_pose(robot_pose, current_speed_lin, current_speed_rot, passed_time);
  const tf2::Transform laser_pose_change = laser_position_tf.inverse() * (changed_robot_pose * laser_position_tf);

  return deskew_point(laser_pose_change, tf2::Vector3(x_cord, y_cord, 0));
}
void DeskewLaserScan::publish_pcl_data(const pcl::PointCloud<PointType>::Ptr &this_cloud,const  rclcpp::Time &thisStamp, const std::string &thisFrame)
{
  sensor_msgs::msg::PointCloud2 tempCloud;
  pcl::toROSMsg(*this_cloud, tempCloud);
  tempCloud.header.stamp = thisStamp;
  tempCloud.header.frame_id = thisFrame;
  pub_1->publish(tempCloud);
}
const tf2::Transform DeskewLaserScan::move_robot_pose(const tf2::Transform &current_robot_pose,const  float &linear_velocity,const float &angular_velocity,const float &duration)
{
  /*
    DIFF DRIVE KINEMATICS, NEGLECTED FOR THIS SMALL MOTION
    float shuffle = (linear_velocity / abs(angular_velocity)) * sqrt(2 * (1 - cos(angular_velocity * duration)));
  */
  const float shuffle = linear_velocity * duration;
  tf2::Quaternion rotation = tf2::Quaternion();
  rotation.setRPY(0, 0, angular_velocity * duration);
  const tf2::Transform change_transform = tf2::Transform(rotation, tf2::Vector3(shuffle, 0, 0));
  return current_robot_pose * change_transform;
}
const tf2::Vector3 DeskewLaserScan::deskew_point(const tf2::Transform &lazer_pose_change,const  tf2::Vector3 &relevant_lazer_point)
{
  return quatRotate(lazer_pose_change.getRotation(), relevant_lazer_point + lazer_pose_change.getOrigin());
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  rclcpp::executors::MultiThreadedExecutor multiThreadExec(rclcpp::ExecutorOptions(), 2);
  auto main = std::make_shared<DeskewLaserScan>(options);
  multiThreadExec.add_node(main);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Deskew Started.\033[0m");

  multiThreadExec.spin();

  rclcpp::shutdown();
  return 0;
}