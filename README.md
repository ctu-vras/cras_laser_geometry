# cras_laser_geometry

This package provides a nodelet for converting laser scans to point clouds. 

# Nodelets

## cras_laser_geometry/scan_to_point_cloud

### Topics subscribed
- `scan` (`sensor_msgs::LaserScan`) Laser scan.

### Topics published
- `cloud` (`sensor_msgs::PointCloud2`) Point cloud converted from the laser scan.

### Parameters
- `target_frame` (`str`) Target frame to transform point cloud to.
- `wait_for_transform` (`double`) Seconds to wait until transforms is available.
- `channel_options` (`int`) Channels to extract from laser scans.
- `scan_queue` (`int`) Scan queue size.
- `point_cloud_queue` (`int`) Point cloud queue size.
