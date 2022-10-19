#include "ros2_kinect/kinect_node.hpp"
#include "libfreenect.h"
#include "libfreenect_sync.h"
#include "drivers/freenect_driver.hpp"
#include <opencv2/highgui/highgui_c.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

namespace Kinect
{
  KinectNode::KinectNode(const rclcpp::NodeOptions &options)
    : Node("kinect_node", options)
  {
    rgb_img_publisher = this->create_publisher<sensor_msgs::msg::Image>(
      "/kinect/rgb_img", rclcpp::SensorDataQoS());
    depth_img_publisher = this->create_publisher<sensor_msgs::msg::Image>(
      "/kinect/depth_img", rclcpp::SensorDataQoS());
    kinect_info_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/kinect/kinect_info", rclcpp::SensorDataQoS());

    //int depth_publish_time = (1/depth_publish_rate)*1000;
    //depth_publish_timer = this->create_wall_timer(
    //  std::chrono::milliseconds(depth_publish_time), std::bind(&KinectNode::depthPublisher, this));

    int rgb_publish_time = (1/rgb_publish_rate)*1000;
    rgb_publish_timer = this->create_wall_timer(
      std::chrono::milliseconds(rgb_publish_time), std::bind(&KinectNode::rgbPublisher, this));

    int info_publish_time = (1/rgb_publish_rate)*1000;
    info_publish_timer = this->create_wall_timer(
      std::chrono::milliseconds(info_publish_time), std::bind(&KinectNode::kinectInfoPublisher, this));
  }

  IplImage *KinectNode::freenect_sync_get_depth_cv(int index)
  {
    static IplImage *image = 0;
    static char *data = 0;
    if (!image) image = cvCreateImageHeader(cvSize(640,480), 16, 1);
    unsigned int timestamp;
    if (freenect_sync_get_depth((void**)&data, &timestamp, index, FREENECT_DEPTH_11BIT))
        return NULL;
    cvSetData(image, data, 640*2);
    return image;
  }

  IplImage *KinectNode::freenect_sync_get_rgb_cv(int index)
  {
    static IplImage *image = 0;
    static char *data = 0;
    if (!image) image = cvCreateImageHeader(cvSize(640,480), 8, 3);
    unsigned int timestamp;
    if (freenect_sync_get_video((void**)&data, &timestamp, index, FREENECT_VIDEO_RGB))
        return NULL;
    cvSetData(image, data, 640*3);
    return image;
  }

  void KinectNode::rgbPublisher()
  {
    while (cvWaitKey(10) < 0) 
    {
      IplImage *image = freenect_sync_get_rgb_cv(0);
      if (!image) 
      {
        std::cout << "Error: Kinect not connected!" << std::endl;
      }
      else
      {
        cv::Mat img = cv::cvarrToMat(image);

        auto header = std_msgs::msg::Header{};
        header.frame_id = "camera_link";
        header.stamp = this->get_clock()->now();

        auto out = cv_bridge::CvImage{header, "bgr8", img};
        auto message = sensor_msgs::msg::Image{};
        out.toImageMsg(message);

        rgb_img_publisher->publish(message);
      }
    }
  }

  void KinectNode::depthPublisher()
  {

  }

  void KinectNode::kinectInfoPublisher()
  {
    sensor_msgs::msg::CameraInfo kinectInfo;
    kinectInfo.header.frame_id = "kinect_link";
    kinectInfo.height = 640;
    kinectInfo.width = 480;

    kinect_info_publisher->publish(kinectInfo);
  }
}
