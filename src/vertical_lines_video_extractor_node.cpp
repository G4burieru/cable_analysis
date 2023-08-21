#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;

class Vertical_Lines_Extractor {
  ros::NodeHandle nh_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  Vertical_Lines_Extractor() : it_(nh_) {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1,
                               &Vertical_Lines_Extractor::imageCb, this);
    image_pub_ = it_.advertise("/vertical_lines_extractor/raw_image", 1);
  }

  ~Vertical_Lines_Extractor() {}

  void imageCb(const sensor_msgs::ImageConstPtr &msg) {

    cv_bridge::CvImagePtr cv_ptr;
    namespace enc = sensor_msgs::image_encodings;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if (cv_ptr->image.rows > 400 && cv_ptr->image.cols > 600) {
      image_pub_.publish(cv_ptr->toImageMsg());
      vertical_lines_extractor(cv_ptr->image);
    }
  }

  void vertical_lines_extractor(cv::Mat img) {

    Mat src, src_gray;
    Mat dst, bw;

    img.copyTo(src);

    if (src.channels() == 3) {
      cvtColor(src, src_gray, COLOR_BGR2GRAY);
    }

    else {
      src_gray = src;
    }

    adaptiveThreshold(~src_gray, bw, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY,
                      15, -2);

    dst = bw.clone();

    int vertical_size = dst.rows / 30;

    Mat verticalStructure =
        getStructuringElement(MORPH_RECT, Size(1, vertical_size));

    erode(dst, dst, verticalStructure, Point(-1, -1));
    dilate(dst, dst, verticalStructure, Point(-1, -1));

    imshow("vertical_without_dilate", dst);

    Mat kernel = Mat::ones(3, 3, CV_8UC1);
    dilate(dst, dst, kernel);

    blur(dst, dst, Size(2, 2));

    imshow("vertical", dst);
    waitKey(1);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "Vertical_Lines_Extractor");
  Vertical_Lines_Extractor extractor;
  ros::spin();
  return 0;
}
