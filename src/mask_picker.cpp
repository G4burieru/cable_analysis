#include <mask_picker.h>

namespace cable_analysis
{

  /* onInit() //{ */

  void MaskPicker::onInit()
  {

    /* obtain node handle */
    ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

    /* waits for the ROS to publish clock */
    ros::Time::waitForValid();

    // | ------------------- load ros parameters ------------------ |
    bool loaded_successfully = true;

    loaded_successfully &= nh.getParam("rate/check_subscribers", _rate_timer_check_subscribers_);
    loaded_successfully &= nh.getParam("acceptable_interval_last_image", _acceptable_interval_last_image_);

    if (!loaded_successfully)
    {
      ROS_ERROR("[MaskPicker]: failed to load non-optional parameters");
      ros::shutdown();
    }

    // | --------------------------- GUI -------------------------- |
    int flags = cv::WINDOW_NORMAL | cv::WINDOW_FREERATIO | cv::WINDOW_GUI_EXPANDED;
    cv::namedWindow("Mask Picker", flags);

    /* create a trackbar for user to enter thresholders */
    cv::createTrackbar("Min H:", "Mask Picker", &lower_bound_H_, max_hue_);
    cv::createTrackbar("Max H:", "Mask Picker", &upper_bound_H_, max_hue_);

    cv::createTrackbar("Min S:", "Mask Picker", &lower_bound_S_, max_saturation_);
    cv::createTrackbar("Max S:", "Mask Picker", &upper_bound_S_, max_saturation_);

    // | ----------------- initialize subscribers ----------------- |

    /* initialize the image transport, needs node handle */
    image_transport::ImageTransport it(nh);

    sub_camera_color_ = it.subscribe("camera_color_in", 1, &MaskPicker::callbackSubscriberCameraColor, this);

    // | ------------------- initialize timers -------------------- |
    timer_check_subscribers_ = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_),
                                              &MaskPicker::callbackTimerCheckSubscribers, this);

    ROS_INFO_ONCE("[MaskPicker]: initialized");

    is_initialized_ = true;
    got_camera_image_ = false;
  }

  //}

  // | ------------------- subscribers callbacks ------------------ |

  /* callbackSubscriberCameraColor //{ */

  void MaskPicker::callbackSubscriberCameraColor(const sensor_msgs::ImageConstPtr &msg)
  {

    if (!is_initialized_)
    {
      return;
    }

    /* turn on camera image flag */
    got_camera_image_ = true;
    time_last_image_ = ros::Time::now();

    /* create pointer to the image */
    const cv_bridge::CvImagePtr bridge_image_copy_ptr = cv_bridge::toCvCopy(msg, "bgr8");

    /* blur image to reduce the noise */
    cv::Mat img_blur;
    cv::medianBlur(bridge_image_copy_ptr->image, img_blur, 3);

    /* convert image to HSV */
    cv::Mat img_hsv;
    cv::cvtColor(img_blur, img_hsv, cv::COLOR_BGR2HSV);

    /* apply color segmentation */
    cv::Mat img_threshold;
    cv::inRange(img_hsv, cv::Scalar(lower_bound_H_, lower_bound_S_, 0), cv::Scalar(upper_bound_H_, upper_bound_S_, 255), img_threshold);

    /* transform the result of the segmentation to BGR */
    cv::Mat mask;
    cv::cvtColor(img_threshold, mask, cv::COLOR_GRAY2BGR);

    /* bitwise operation between the original image and the mask */
    cv::Mat img_result = bridge_image_copy_ptr->image & mask;

    /* show result */
    cv::imshow("Mask Picker", img_result);

    cv::waitKey(1);
  }

  //}

  // | --------------------- timer callbacks ---------------------- |

  /* callbackTimerCheckSubscribers() //{ */

  void MaskPicker::callbackTimerCheckSubscribers([[maybe_unused]] const ros::TimerEvent &te)
  {

    if (!is_initialized_)
    {
      return;
    }

    if (!got_camera_image_)
    {
      /* print a warning if the node has not received a message after launch */
      ROS_WARN_THROTTLE(2, "[MaskPicker]: not received camera image since node launch");
    }
    else
    {
      double interval_last_image = (ros::Time::now() - time_last_image_).toSec();

      /* print a warning if there is a considerable delay between received messages */
      if (interval_last_image > _acceptable_interval_last_image_)
      {
        ROS_WARN_THROTTLE(2, "[MaskPicker] last image was received more than %lf seconds ago",
                          _acceptable_interval_last_image_);
      }
    }
  }

  //}

} // namespace cable_analysis

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cable_analysis::MaskPicker, nodelet::Nodelet)