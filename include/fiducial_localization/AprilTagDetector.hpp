#ifndef APRILTAG_DETECTOR_HPP
#define APRILTAG_DETECTOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.h"
#include "std_msgs/msg/header.h"

#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
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

  cv::Mat _image_detections;
  cv::Mat _image_detections_gray;
  cv_bridge::CvImagePtr _cv_image_prt;

  apriltag_family_t *_tag_family;
  apriltag_detector_t *_tag_detector;

};

#endif  // APRILTAG_DETECTOR_HPP
