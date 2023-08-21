#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
Mat src, src_gray;
Mat dst, detected_edges;
Mat gaussiancopy;
int lowThreshold = 0;
const int max_lowThreshold = 100;
const int ratio = 3;
const int kernel_size = 3;
const char* window_name = "Edge Map";

class Edge_Detector {
  ros::NodeHandle nh_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber     image_sub_;
  image_transport::Publisher      image_pub_;

public:
  Edge_Detector() : it_(nh_) {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &Edge_Detector::imageCb, this);
    image_pub_ = it_.advertise("/edge_detector/raw_image", 1);
  }

  ~Edge_Detector() {
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImagePtr cv_ptr;
    namespace enc = sensor_msgs::image_encodings;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // cv::Mat test = cv_ptr->image;

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 400 && cv_ptr->image.cols > 600) {

      canny_edge_detection(cv_ptr->image);
      image_pub_.publish(cv_ptr->toImageMsg());
      imshow( window_name, dst );
    }
  }

    static void CannyThreshold(int, void*){
        blur( src_gray, detected_edges, Size(11,11) );
        Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
        dst = Scalar::all(0);
        src.copyTo( dst, detected_edges);
    }

    void canny_edge_detection(cv::Mat img) {

        img.copyTo(src);

        dst.create(src.size(), src.type());
        cvtColor(src, src_gray, CV_BGR2GRAY);
        namedWindow(window_name, WINDOW_AUTOSIZE);
        createTrackbar("Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold);
        CannyThreshold(0, 0);
        waitKey(1);
    }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "Edge_Detector");
  Edge_Detector ic;
  ros::spin();
  return 0;
}
