#include <laser_geometry/laser_geometry.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cras_cpp_common/nodelet_utils.hpp>

namespace cras_laser_geometry {

/**
 * @brief A nodelet converting laser scans to point clouds.
 *
 * This nodelet converts scans (i.e., sensor_msgs::LaserScan messages) to point clouds (i.e., to
 * sensor_msgs::PointCloud2 messages).
 *
 */
class ScanToPointCloud: public cras::Nodelet {
public:
  ScanToPointCloud();
  ~ScanToPointCloud() override = default;
  void onInit() override;
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scanPtr);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  std::string targetFrame;
  ros::Duration waitForTransform;
  /**
   * @brief channelOptions Channels to extract.
   *
   * Channels to extract, see laser_geometry.h for details.
   * 0x00 - no channels enabled,
   * 0x01 - enable intensity (default),
   * 0x02 - enable index (default),
   * 0x04 - enable distance,
   * 0x08 - enable stamps,
   * 0x10 - enable viewpoint.
   */
  int channelOptions;
  /** @brief Scan queue size. */
  uint32_t scanQueue;
  /** @brief Point cloud queue size. */
  uint32_t pointCloudQueue;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener transformListener;
  ros::Subscriber scanSubscriber;
  laser_geometry::LaserProjection projector;
  ros::Publisher pointCloudPublisher;
};

ScanToPointCloud::ScanToPointCloud():
  targetFrame(""),
  waitForTransform(0.0),
  channelOptions(laser_geometry::channel_option::Default),
  scanQueue(10),
  pointCloudQueue(10),
  tfBuffer(),
  transformListener(tfBuffer) {
}

void ScanToPointCloud::onInit() {
  NODELET_INFO("ScanToPointCloud::onInit: Initializing...");

  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &pnh = getPrivateNodeHandle();

  // Process parameters.
  targetFrame = this->getParam(pnh, "target_frame", targetFrame);
  waitForTransform = this->getParam(pnh, "wait_for_transform", ros::Duration(0.01), "s");
  channelOptions = this->getParam(pnh, "channel_options", channelOptions);
  scanQueue = this->getParam(pnh, "scan_queue", scanQueue);
  pointCloudQueue = this->getParam(pnh, "point_cloud_queue", pointCloudQueue);

  // Subscribe scan topic.
  std::string scanTopic = nh.resolveName("scan", true);
  NODELET_INFO("ScanToPointCloud::onInit: Subscribing scan %s.", scanTopic.c_str());
  scanSubscriber = nh.subscribe<sensor_msgs::LaserScan>(scanTopic, scanQueue, &ScanToPointCloud::scanCallback, this);

  // Advertise scan point cloud.
  std::string cloudTopic = nh.resolveName("cloud", true);
  NODELET_INFO("ScanToPointCloud::onInit: Advertising point cloud %s.", cloudTopic.c_str());
  pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>(cloudTopic, pointCloudQueue, false);
}

void ScanToPointCloud::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scanPtr) {
  // ros::Time t1 = ros::Time::now();
  // NODELET_DEBUG("ScanToPointCloud::scanCallback: Point cloud received (%lu points).", scanPtr->ranges.size());
  // Wait for the transform if the target frame differs from that of the scan.
  this->updateThreadName();
  std::string frame = !targetFrame.empty() ? targetFrame : scanPtr->header.frame_id;
  std::string tfError;
  if (frame != scanPtr->header.frame_id
      && !tfBuffer.canTransform(frame, scanPtr->header.frame_id,
                                scanPtr->header.stamp + ros::Duration(scanPtr->scan_time),
                                ros::Duration(waitForTransform), &tfError)) {
    NODELET_WARN("ScanToPointCloud::scanCallback: Cannot transform from %s to %s at %.2f s. Error %s",
                 scanPtr->header.frame_id.c_str(), frame.c_str(), scanPtr->header.stamp.toSec(), tfError.c_str());
    return;
  }
  sensor_msgs::PointCloud2::Ptr cloud2(new sensor_msgs::PointCloud2);
  try {
    projector.transformLaserScanToPointCloud(frame, *scanPtr, *cloud2, tfBuffer, -1.0, channelOptions);
    pointCloudPublisher.publish(cloud2);
    // NODELET_DEBUG("ScanToPointCloud::scanCallback: Converting scan to point cloud: %.3f s.",
    // (ros::Time::now() - t1).toSec());
  } catch(tf2::TransformException& ex) {
    ROS_ERROR("ScanToPointCloud::scanCallback: Transform exception: %s.", ex.what());
    return;
  }
}

} /* namespace cras_laser_geometry */

PLUGINLIB_EXPORT_CLASS(cras_laser_geometry::ScanToPointCloud, nodelet::Nodelet) // NOLINT(cert-err58-cpp)
