#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

void show_wait_destroy(const char *winname, cv::Mat img);

using namespace std;
using namespace cv;

int main(int argc, char **argv) {
  CommandLineParser parser(argc, argv, "{@input | cabos.png | input image}");
  Mat src = imread("/home/gabs/Pictures/2.png");

  if (src.empty()) {
    cout << "Could not open or find the image!\n" << endl;
    cout << "Usage: " << argv[0] << " <Input image>" << endl;
    return -1;
  }
  // Show source image
  imshow("src", src);
  // Transform source image to gray if it is not already
  Mat gray;

  if (src.channels() == 3) {
    cvtColor(src, gray, COLOR_BGR2GRAY);
  }

  else {
    gray = src;
  }

  // Show gray image
  show_wait_destroy("gray", gray);

  blur(gray, gray, Size(7, 7));

  // Apply adaptiveThreshold at the bitwise_not of gray, notice the ~ symbol
  Mat bw;
  adaptiveThreshold(~gray, bw, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 15,
                    -2);

  // Show binary image
  show_wait_destroy("binary", bw);

  Mat mask;
  cv::cvtColor(bw, mask, cv::COLOR_GRAY2BGR);

  cv::Mat img_result = src & mask;

  /* show result */
  cv::imshow("Mask Picker", img_result);

  // Create the images that will use to extract the horizontal and vertical
  // lines
  Mat horizontal = bw.clone();
  Mat vertical = bw.clone();

  // Specify size on horizontal axis
  int horizontal_size = horizontal.cols / 30;

  // Create structure element for extracting horizontal lines through morphology
  // operations
  Mat horizontalStructure =
      getStructuringElement(MORPH_RECT, Size(horizontal_size, 1));

  // Apply morphology operations
  erode(horizontal, horizontal, horizontalStructure, Point(-1, -1));
  dilate(horizontal, horizontal, horizontalStructure, Point(-1, -1));

  // Show extracted horizontal lines
  show_wait_destroy("horizontal", horizontal);

  // Specify size on vertical axis
  int vertical_size = vertical.rows / 30;

  // Create structure element for extracting vertical lines through morphology
  // operations
  Mat verticalStructure =
      getStructuringElement(MORPH_RECT, Size(1, vertical_size));

  // Apply morphology operations
  erode(vertical, vertical, verticalStructure, Point(-1, -1));
  dilate(vertical, vertical, verticalStructure, Point(-1, -1));

  // Show extracted vertical lines
  show_wait_destroy("vertical", vertical);

  RNG rng(12345);
  vector<vector<Point>> contours_2;
  vector<Vec4i> hierarchy_2;

  findContours(vertical, contours_2, hierarchy_2, RETR_TREE,
               CHAIN_APPROX_SIMPLE);

  Mat drawing_2 = Mat::zeros(vertical.size(), CV_8UC3);
  for (size_t i = 0; i < contours_2.size(); i++) {
    Scalar color =
        Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    drawContours(drawing_2, contours_2, (int)i, color, 2, LINE_8, hierarchy_2,
                 0);
  }
  imshow("Contours", drawing_2);

  // Inverse vertical image
  bitwise_not(vertical, vertical);
  show_wait_destroy("vertical_bit", vertical);

  // Extract edges and smooth image according to the logic
  // 1. extract edges
  // 2. dilate(edges)
  // 3. src.copyTo(smooth)
  // 4. blur smooth img
  // 5. smooth.copyTo(src, edges)
  // Step 1
  Mat edges;
  adaptiveThreshold(vertical, edges, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY,
                    3, -2);
  show_wait_destroy("edges", edges);

  vector<vector<Point>> contours_1;
  vector<Vec4i> hierarchy_1;

  findContours(edges, contours_1, hierarchy_1, RETR_TREE, CHAIN_APPROX_SIMPLE);

  Mat drawing_1 = Mat::zeros(edges.size(), CV_8UC3);
  for (size_t i = 0; i < contours_1.size(); i++) {
    Scalar color =
        Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    drawContours(drawing_1, contours_1, (int)i, color, 2, LINE_8, hierarchy_1,
                 0);
  }
  imshow("Contours_1", drawing_1);

  // Step 2
  Mat kernel = Mat::ones(2, 2, CV_8UC1);
  dilate(edges, edges, kernel);
  show_wait_destroy("dilate", edges);

  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;

  findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

  Mat drawing = Mat::zeros(edges.size(), CV_8UC3);
  for (size_t i = 0; i < contours.size(); i++) {
    Scalar color =
        Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    drawContours(drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0);
  }
  imshow("Contours_2", drawing);

  // Step 3
  Mat smooth;
  vertical.copyTo(smooth);

  // Step 4
  blur(smooth, smooth, Size(2, 2));

  // Step 5
  smooth.copyTo(vertical, edges);

  // Show final result
  show_wait_destroy("smooth - final", vertical);
  return 0;
}

void show_wait_destroy(const char *winname, cv::Mat img) {
  imshow(winname, img);
  moveWindow(winname, 500, 0);
  waitKey(0);
  destroyWindow(winname);
}
