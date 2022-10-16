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
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

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
		void setup();

	};

	// Ros2 Publishers
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_img_publisher;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_img_publisher;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ir_img_publisher;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr kinect_info_publisher;


}  // namespace Kinect