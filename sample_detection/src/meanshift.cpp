#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/photo/photo.hpp"

using namespace cv;
using namespace std;

Mat img, res, hsv, element, mask;
vector<Mat> hsv_planes;
int sp = 20.0, sr=20.0;

void spCallback(int,void*);
static void onMouse( int event, int x, int y, int, void* );


int main(int argc, char** argv)
{
   img = imread( argv[1] );
   if (img.empty())
       return -1;

   mask = cv::Mat::zeros(img.rows + 2, img.cols + 2, CV_8U);

   cvtColor(img, hsv, CV_BGR2HSV);
   split(hsv, hsv_planes );
/*
   hsv_planes[2] = Scalar(255);
   threshold(hsv_planes[2], hsv_planes[2], 50, 255, CV_THRESH_BINARY_INV);
   inpaint(img, hsv_planes[2], img, 3.0, CV_INPAINT_NS);
*/

   // brighten up dark spots
   // FIXME: assuming no black objects
   hsv_planes[2] = ~(hsv_planes[2]);
   threshold(hsv_planes[2], hsv_planes[2], 200, 0, CV_THRESH_TRUNC);
   hsv_planes[2] = ~(hsv_planes[2]);
   merge(hsv_planes, hsv);
   cvtColor(hsv, img, CV_HSV2BGR);

   namedWindow( "window", CV_WINDOW_NORMAL );
   namedWindow( "maskwindow", CV_WINDOW_NORMAL );
   createTrackbar( "Spatial radius", "window", &sp, 255, NULL );
   createTrackbar( "Color radius", "window", &sr, 255, NULL );
   setMouseCallback( "window", onMouse, 0 );

   //pyrMeanShiftFiltering( img, res, sp, sr, 3);
//   imwrite("output.png", res);
   imshow( "window", img );

   while(waitKey() != 27); // 27 = ascii value of ESC

   return 0;
}

void spCallback(int a, void * b)
{
    pyrMeanShiftFiltering( img, res, (float)sp, (float)sr, 3);
//    pyrSegmentation(res, res, )
    cvtColor(res, hsv, CV_HSV2BGR);
    split(hsv, hsv_planes );
    imshow( "window", hsv_planes[0] );
//    imshow( "window", res );
}

static void onMouse( int event, int x, int y, int, void* )
{
	switch (event) {
		case CV_EVENT_RBUTTONUP:
			cvtColor(res, hsv, CV_HSV2BGR);
			split(hsv, hsv_planes );
			floodFill(hsv, mask, Point(x,y), 255, 0,
					cv::Scalar(), cv::Scalar(10,150,250),
					4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);
			break;
		case CV_EVENT_LBUTTONUP:
			spCallback(0, NULL);
			break;
		default:
			break;
	}

	imshow( "maskwindow", mask );
}
