#include "ros2_kinect/kinect_node.hpp"
#include <boost/algorithm/string/replace.hpp>

namespace Kinect
{
  KinectNode::KinectNode(const rclcpp::NodeOptions &options)
		: Node("kinect_node", options)
  {
		rgb_img_publisher = this->create_publisher<sensor_msgs::msg::Image>(
			"/kinect/rgb_img", rclcpp::SensorDataQoS());
		depth_img_publisher = this->create_publisher<sensor_msgs::msg::Image>(
			"/kinect/depth_img", rclcpp::SensorDataQoS());
		ir_img_publisher = this->create_publisher<sensor_msgs::msg::Image>(
			"/kinect/ir_img", rclcpp::SensorDataQoS());
		img_test_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
			"/kinect/kinect_info", rclcpp::SensorDataQoS());
  }


  void KinectNode::setup()
  {

  }
}
