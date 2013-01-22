#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/photo/photo.hpp"

using namespace cv;
using namespace std;

Mat img, res, hsv, element;
vector<Mat> hsv_planes;
int sp = 50.0, sr=40.0;

void spCallback(int,void*);

int main(int argc, char** argv)
{
   img = imread( argv[1] );
   if (img.empty())
       return -1;

   cvtColor(img, hsv, CV_BGR2HSV);
   split(hsv, hsv_planes );
/*
   hsv_planes[2] = Scalar(255);
   threshold(hsv_planes[2], hsv_planes[2], 50, 255, CV_THRESH_BINARY_INV);
   inpaint(img, hsv_planes[2], img, 3.0, CV_INPAINT_NS);
*/

   hsv_planes[2] = ~(hsv_planes[2]);
   threshold(hsv_planes[2], hsv_planes[2], 200, 0, CV_THRESH_TRUNC);
   hsv_planes[2] = ~(hsv_planes[2]);
   merge(hsv_planes, hsv);
   cvtColor(hsv, img, CV_HSV2BGR);

   namedWindow( "window", CV_WINDOW_NORMAL );
   createTrackbar( "Spatial radius", "window", &sp, 255, spCallback );
   createTrackbar( "Color radius", "window", &sr, 255, spCallback );

   //pyrMeanShiftFiltering( img, res, sp, sr, 3);
//   imwrite("output.png", res);
   imshow( "window", img );

   waitKey();

   return 0;
}

void spCallback(int a, void * b)
{
    pyrMeanShiftFiltering( img, res, (float)sp, (float)sr, 3);
//    cvtColor(res, res, CV_HSV2BGR);
    imshow( "window", res );
}

