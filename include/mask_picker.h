#pragma once
#ifndef MASK_PICKER_H
#define MASK_PICKER_H

/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* some OpenCV includes */
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/* ROS includes for working with OpenCV and images */
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

/* include some sensor messages */
#include <sensor_msgs/Image.h>

//}

namespace cable_analysis
{

  /* class MaskPicker //{ */

  class MaskPicker : public nodelet::Nodelet
  {

  public:
    virtual void onInit();

  private:
    /* flags */
    bool is_initialized_ = false;
    bool got_camera_image_ = false;

    /* ROS parameters */
    int _rate_timer_check_subscribers_;
    double _acceptable_interval_last_image_;

    // | ------------------- subscribers callbacks ------------------ |
    void callbackSubscriberCameraColor(const sensor_msgs::ImageConstPtr &msg);
    image_transport::Subscriber sub_camera_color_;

    int const max_hue_ = 255;
    int const max_saturation_ = 255;
    int lower_bound_H_ = 105;
    int lower_bound_S_ = 0;
    int upper_bound_H_ = 140;
    int upper_bound_S_ = 120;

    // | --------------------- timer callbacks ---------------------- |
    void callbackTimerCheckSubscribers(const ros::TimerEvent &te);
    ros::Timer timer_check_subscribers_;
    ros::Time time_last_image_;
  };

  //}

} // namespace cable_analysis

#endif