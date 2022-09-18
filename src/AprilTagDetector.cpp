#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "fiducial_localization/AprilTagDetector.hpp"

#include "apriltag/tag36h11.h"

#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>

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

  _image_timer = this->create_wall_timer(100ms, std::bind(&AprilTagDetector::publishImageDetections, this));

  _tag_family = tag36h11_create();
  _tag_detector = apriltag_detector_create();
  apriltag_detector_add_family(_tag_detector, _tag_family);

}

AprilTagDetector::~AprilTagDetector()
{
}

void AprilTagDetector::publishImageDetections()
{
  cv::cvtColor(_cv_image_prt->image, _image_detections_gray, CV_BGR2GRAY);
  _image_detections = _cv_image_prt->image;

  image_u8_t img = {_image_detections_gray.cols,
                    _image_detections_gray.rows,
                    _image_detections_gray.cols,
                    _image_detections_gray.data
                    };

  zarray_t *detections = apriltag_detector_detect(_tag_detector, &img);

  for (int i=0; i<zarray_size(detections); i++)
  {
    apriltag_detection_t *single_detection;
    zarray_get(detections, i, &single_detection);
    cv::line(_image_detections, 
             cv::Point(single_detection->p[0][0], single_detection->p[0][1]),
             cv::Point(single_detection->p[1][0], single_detection->p[1][1]),
             cv::Scalar(0, 0xff, 0), 2);
    cv::line(_image_detections, 
             cv::Point(single_detection->p[0][0], single_detection->p[0][1]),
             cv::Point(single_detection->p[3][0], single_detection->p[3][1]),
             cv::Scalar(0, 0, 0xff), 2);
    cv::line(_image_detections, 
             cv::Point(single_detection->p[1][0], single_detection->p[1][1]),
             cv::Point(single_detection->p[2][0], single_detection->p[2][1]),
             cv::Scalar(0xff, 0, 0), 2);
    cv::line(_image_detections, 
             cv::Point(single_detection->p[2][0], single_detection->p[2][1]),
             cv::Point(single_detection->p[3][0], single_detection->p[3][1]),
             cv::Scalar(0xff, 0, 0), 2);
  }

  _cv_image_prt->image = _image_detections;
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
