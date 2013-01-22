#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

Mat img, res, element;
vector<Mat> hsv_planes;
int sp = 100.0, sr=40.0;

void spCallback(int,void*);

int main(int argc, char** argv)
{
   img = imread( argv[1] );
   if (img.empty())
       return -1;
//   cvtColor(img, img, CV_BGR2HSV);
//   split(img, hsv_planes );
//   hsv_planes[2] = Scalar(255);
//   merge(hsv_planes, img);
   namedWindow( "window", CV_WINDOW_NORMAL );
   createTrackbar( "Low thresh", "window", &sp, 255, spCallback );
   createTrackbar( "High thresh", "window", &sr, 255, spCallback );

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

