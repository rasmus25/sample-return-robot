#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  //for blob detection
  vector<Mat> color_planes;
  Mat out;
  SimpleBlobDetector::Params params;
  SimpleBlobDetector * blobDetector;
  vector<KeyPoint> keyPoints;
  vector< vector <Point> > contours;
  vector< vector <Point> > approxContours;

public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("output_image", 1);
    image_sub_ = it_.subscribe("input_image", 1, &ImageConverter::imageCb, this);

    //for blob detection
    params.minDistBetweenBlobs = 10.0;  // minimum 10 pixels between blobs
    params.filterByArea = true;         // filter my blobs by area of blob
    params.minArea = 100;              // min 100 pixels squared
    params.blobColor = 255;
//    params.maxArea = 5000.0;             // max 500 pixels squared
//	params.minThreshold = 0;
//	params.maxThreshold = 5;
//	params.thresholdStep = 5;
//
//	params.minArea = 10;
//	params.minConvexity = 0.3;
//	params.minInertiaRatio = 0.01;
//
//	params.maxArea = 8000;
//	params.maxConvexity = 10;
//
//	params.filterByColor = false;
//	params.filterByCircularity = false;

	blobDetector = new SimpleBlobDetector( params );
	blobDetector->create("SimpleBlob");

    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::split(cv_ptr->image, color_planes);
	blobDetector->detect( color_planes[2], keyPoints);
	drawKeypoints( cv_ptr->image, keyPoints, out);

    cv::imshow(WINDOW, out);
    cv::waitKey(3);
    
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

