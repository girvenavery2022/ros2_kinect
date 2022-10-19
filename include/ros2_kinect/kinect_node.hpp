#pragma once

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core_c.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "drivers/freenect_driver.hpp"

namespace Kinect
{
  class KinectNode : public rclcpp::Node
  {
  public:
    explicit KinectNode(const rclcpp::NodeOptions &options);

  private:

    /**
    * @brief
    *
    */
    void depthPublisher();

    /**
    * @brief
    *
    */
    void rgbPublisher();

    /**
    * @brief
    *
    */
    void kinectInfoPublisher();

    /**
     * @brief 
     * 
     * @param index 
     * @return IplImage* 
     */
	  IplImage *freenect_sync_get_depth_cv(int index);

    /**
     * @brief 
     * 
     * @param index 
     * @return IplImage* 
     */
	  IplImage *freenect_sync_get_rgb_cv(int index);

    // Ros2 Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_img_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_img_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr kinect_info_publisher;

    rclcpp::TimerBase::SharedPtr depth_publish_timer;
    rclcpp::TimerBase::SharedPtr rgb_publish_timer;
    rclcpp::TimerBase::SharedPtr info_publish_timer;
    
    float depth_publish_rate{};
    float rgb_publish_rate{10};
  };
}  // namespace Kinect