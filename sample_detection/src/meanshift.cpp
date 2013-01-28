#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/photo/photo.hpp"
#include <string>
#include <sstream>

using namespace cv;
using namespace std;

Mat img, res, hsv, element, mask, thresholded_image, threshold_helper;
vector<Mat> hsv_planes;
int sp = 20, sr=20;
int lower_threshold = 50, upper_threshold = 75;

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
   createTrackbar( "Spatial radius", "window", &sp, 255, NULL );
   createTrackbar( "Color radius", "window", &sr, 255, NULL );
   setMouseCallback( "window", onMouse, 0 );

   namedWindow( "maskwindow", CV_WINDOW_NORMAL );
   createTrackbar( "Lower thresh", "maskwindow", &lower_threshold, 180, NULL );
   createTrackbar( "Upper thresh", "maskwindow", &upper_threshold, 180, NULL );

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
    cvtColor(res, hsv, CV_BGR2HSV);
    split(hsv, hsv_planes );
    imshow( "window", hsv_planes[0] );
//    imshow( "window", res );
}

static void onMouse( int event, int x, int y, int, void* )
{
	switch (event) {
		case CV_EVENT_RBUTTONUP:
//			cvtColor(res, hsv, CV_BGR2HSV);
			split(img, hsv_planes );
//			floodFill(hsv, mask, Point(x,y), 255, 0,
//					cv::Scalar(), cv::Scalar(20,250,250),
//					4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);
//			imshow( "maskwindow", mask );
			imshow( "maskwindow", hsv_planes[1] );
			break;
		case CV_EVENT_LBUTTONUP:
			spCallback(0, NULL);
			imshow( "maskwindow", mask );
			break;
		case CV_EVENT_MBUTTONUP:
		{
			cvtColor(res, hsv, CV_BGR2HSV);
			split(hsv, hsv_planes );
			threshold(hsv_planes[0], thresholded_image, lower_threshold, 0, CV_THRESH_TOZERO);
			threshold(hsv_planes[0], threshold_helper, upper_threshold, 0, CV_THRESH_TOZERO);
			thresholded_image = thresholded_image - threshold_helper;
			threshold(thresholded_image, thresholded_image, 1, 255, CV_THRESH_BINARY);
			imshow("maskwindow", thresholded_image);

//			stringstream message;
//
//			message << "HSV: "
//					<< (unsigned int) hsv_planes[0].at<unsigned char>(x,y) << ","
//					<< (unsigned int) hsv_planes[1].at<unsigned char>(x,y) << ","
//					<< (unsigned int) hsv_planes[2].at<unsigned char>(x,y);
//			res.copyTo(threshold_helper);
//			putText(threshold_helper, message.str(), cvPoint(60,60),
//					FONT_HERSHEY_COMPLEX_SMALL, 1.0, cvScalar(0,100,250), 2);
//			imshow( "window", threshold_helper );

			// Draw histogram
			/// Establish the number of bins
			int histSize = 180;

			/// Set the range
			float range[] = { 0, 180 } ;
			const float* histRange = { range };

			bool uniform = true; bool accumulate = false;

			Mat hue_hist;

			calcHist( &hsv_planes[0], 1, 0, Mat(), hue_hist, 1, &histSize, &histRange, uniform, accumulate );

			// Draw the histograms for B, G and R
			int hist_w = 512; int hist_h = 400;
			int bin_w = cvRound( (double) hist_w/histSize );

			Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

			/// Normalize the result to [ 0, histImage.rows ]
			normalize(hue_hist, hue_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

			Scalar color;
			/// Draw for each channel
			for( int i = 1; i < histSize; i++ )
			{

				if(i <= lower_threshold || i >= upper_threshold)
					color = Scalar( 255, 0, 0);
				else
					color = Scalar( 0, 0, 255);
				line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hue_hist.at<float>(i-1)) ) ,
							   Point( bin_w*(i), hist_h - cvRound(hue_hist.at<float>(i)) ),
							   color, 2, 8, 0  );
			}

			/// Display
//			namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
//			imshow("calcHist Demo", histImage );
			imshow("window", histImage);
			break;
		}
		default:
			break;
	}


}
