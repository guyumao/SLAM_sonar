#ifndef POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET
#define POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include "fcntl.h"
#include "termios.h"
#include <sys/stat.h>
#include <sys/types.h>
#include "ros/ros.h"
#include "boost/thread/mutex.hpp"

#include "nodelet/nodelet.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/LaserScan.h"


namespace sonar_to_laserscan
{
/**
* Class to process incoming pointclouds into laserscans. Some initial code was pulled from the defunct turtlebot
* pointcloud_to_laserscan implementation.
*/
  class SonarToLaserScanNodelet : public nodelet::Nodelet
  {

  public:
    SonarToLaserScanNodelet();

  private:
    virtual void onInit();

    void cloudCb(const sensor_msgs::LaserScanConstPtr &msg);

    void connectCb();

    void disconnectCb();

    ros::NodeHandle nh_, private_nh_;
    ros::Publisher pub_;
    boost::mutex connect_mutex_;

    boost::shared_ptr<tf2_ros::Buffer> tf2_;
    boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_;

    // ROS Parameters
    unsigned int input_queue_size_;
    std::string target_frame_;
    double tolerance_;
    double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_;
    bool use_inf_;
  };

}  // pointcloud_to_laserscan

#endif  // POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET
