# Laser Scan Deskew ROS2

Package for deskew rotating 2-D lidars scan on mobile platforms based on velocity of the platform. Tested on a diff-dirve platform with odometry and scan message. Used Ydlidar X4 for the results below but probably gona work well with ydlidar G1, G2, G6, GS2, TEA, TG, TminiPro, X2 and X4-pro. Results are with the official [*ydlidar_ros2_driver*](https://github.com/YDLIDAR/ydlidar_ros2_driver/tree/humble) but there is no reason why it won't work with other brand devices, we look forward to your feedback on other devices than ydlidar.

## API

<!-- <p align='center'>
    <img src="images/diagram.drawio.png" alt="drawing" />
</p> -->

### Subscribed topics

| Topic  | Type | Description |
|-----|----|----|
| /diff_cont/odom | `nav_msgs/Odometry` | the odometry message from your vehicle, with valid angular and linear velocities. |
| /scan | `sensor_msgs/LaserScan` | input scan for deskew, if using `sensor_msgs/PointCloud`  will be neglected|
| /pointcloud | `sensor_msgs/PointCloud` | input point cloud for deskew, if using `sensor_msgs/LaserScan` will be neglected |
| /tf | `tf2_msgs/TFMessage` | a valid static transform from your `nav_msgs/Odometry` to `sensor_msgs/LaserScan` or `sensor_msgs/PointCloud` depending on what is used. Assumed stationary lidar will only requested once.|


### Published topics

| Topic  | Type | Description |
|-----|----|----|
| /scan_deskewed  | `sensor_msgs/LaserScan` | deskewed scan to use with mapping, navigation or similar packages.|
| /pointcloud2_deskewed | `sensor_msgs/PointCloud2` | deskewed pointcloud scan to use with mapping, navigation or similar packages.|

### Visual System Representation

[*System Diagram*](images/diagram.drawio.png)
## Resluts

Maps are generated with same bag 5x faster than real-time only with switching the scan topic. Used mapping package [*salam-toolbox*](https://github.com/SteveMacenski/slam_toolbox/tree/ros2).

### With Deskewing

<p align='center'>
    <img src="images/deskewed_map.png" alt="drawing" />
</p>

### Without Deskewing

<p align='center'>
    <img src="images/nodeskew_map.png" alt="drawing" />
</p>

### Green Scan Deskewed

The first picture in red scan we can obviously see that walls that should be parallel are far away from that shape, but thanks to deskew it can be corrected.
The second picture in red scan we can obviously see that walls that should be straight (right below corner) are far away from that shape, but thanks to deskew it can be corrected.

<p align='center'>
    <img src="images/parallel walls green scan deskewed.png" alt="drawing" width="485" height="250"/>
    <img src="images/straigth wall green scan deskewed.png" alt="drawing" width="485" height="250"/>
</p>

## Configuring and Running

### Parameters

You can set the parameters in under config folder in [yaml file](config/ros2_deskew_laser_scan.yaml).

`subscribe_odometry_topic` - Odometry topic with valid velocities relative to base frame. Default is **`/diff_cont/odom`**.

`base_frame` - Base frame of the robot. Child frame of the odom topic. Default is **`base_link`**.

`laser_frame` - Laser frame of the robot. Frame id of the scan message. Default is **`laser_frame`**.

`deskewed_scan_fixed_size` - Fixed size of the output scan. Default is **`620`**.

`invalid_range_is_inf` - Whether ranges that are not in config range of lidar are to be considered as inf. Default is **`true`**.

`deskewed_scan_publish_topic` - Where to publish result deskewed scan. Default is **`/scan_deskewed`**.

`publish_pointcoud2` - Whether to publish scan as pointcloud2 type too. Default is **`true`**.

`deskewed_pointcloud2_publish_topic` - When above param is true, topic to publish. Default is **`/pointcloud2_deskewed`**.

#### Only if Using sensor_msgs/PointCloud as Input

`deskew_from_pointcloud/pointcloud_topic` - Pointcloud topic to use as input.

`deskew_from_pointcloud/min_angle` - User defined since not present in message.

`deskew_from_pointcloud/max_angle` - User defined since not present in message.

`deskew_from_pointcloud/min_range` - User defined since not present in message.

`deskew_from_pointcloud/max_range` - User defined since not present in message.

#### Only if Using sensor_msgs/LaserScan as Input

`deskew_from_scan/use_deskew_from_scan` - If true will neglect pointcloud message above and use scan for deskew. But pointcloud is recommended since the points are ordered according to increasing time from real scan start in  pointcloud. The [*ydlidar_ros2_driver*](https://github.com/YDLIDAR/ydlidar_ros2_driver/tree/humble) outpts an pointcloud topic too and can  be directly used. Thus default is **`false`**.

`deskew_from_scan/subscribe_laser_topic` - If above is true, scan topic to use for deskew. The default is **`/scan`**.

`deskew_from_scan/scan_start_angle` - Since the scan message does not contain actual information about the first taken scan range in the message, we can not now when the first scan range was taken. If the **angle_min** in the scan message corresponds also to the first taken scan range in the message, you can set this to **angle_min**. But that is not the case for [*ydlidar_ros2_driver*](https://github.com/YDLIDAR/ydlidar_ros2_driver/tree/humble) publishing the /scan message for the X4 model. The  default param is a approximate number for the ydlidar X4 **`0.35`** rad and it works fine.