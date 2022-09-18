#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "fiducial_localization/AprilTagDetector.hpp"

#include <memory>
#include <chrono>
#include <cstdlib>

using namespace std::chrono_literals;
using std::placeholders::_1;


AprilTagDetector::AprilTagDetector() : Node("apriltag_detector")
{
  RCLCPP_INFO(this->get_logger(), "AprilTag Detector Node initialized");

  _image_publisher = this->create_publisher<sensor_msgs::msg::Image>("image/detections", 10);
  _image_subscriber = this->create_subscription<sensor_msgs::msg::Image>("camera/image_raw", 10, 
                          std::bind(&AprilTagDetector::imageCallback, this, _1));

  _image_timer = this->create_wall_timer(33ms, std::bind(&AprilTagDetector::publishImageDetections, this));

}

AprilTagDetector::~AprilTagDetector()
{
}

void AprilTagDetector::publishImageDetections()
{
  cv::cvtColor(_cv_image_prt->image, _cv_image_prt->image, CV_BGR2HSV);
  
  _image_publisher->publish(*_cv_image_prt->toImageMsg());
}

void AprilTagDetector::imageCallback(sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    _cv_image_prt = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(const cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagDetector>());
  rclcpp::shutdown();
  return 0;
}
