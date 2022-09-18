#ifndef APRILTAG_DETECTOR_HPP
#define APRILTAG_DETECTOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.h"
#include "std_msgs/msg/header.h"

#include <opencv2/opencv.hpp>
#include <memory>


class AprilTagDetector : public rclcpp::Node
{
public:
  AprilTagDetector();
  ~AprilTagDetector();

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void publishImageDetections();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_subscriber;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_publisher;

  rclcpp::TimerBase::SharedPtr _image_timer;

  cv_bridge::CvImagePtr _cv_image_prt;

};

#endif  // APRILTAG_DETECTOR_HPP
