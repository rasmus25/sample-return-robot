#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/photo/photo.hpp"
#include <list>
#include <string>
#include <sstream>
#include <iostream>

using namespace cv;
using namespace std;

Mat img;

// OpenCV's Rect needs top-left and bottom-right corners
// but I want to use random opposite corners
pair <Point,Point> current_segment;
typedef list < pair <Point,Point> > rect_type;
rect_type image_segments;


static void onMouse( int event, int x, int y, int, void* );

int main(int argc, char** argv)
{
	if(argc < 2)
	{
		cout << "Usage: " << argv[0] << " " << "/path/to/image.png"<<endl;
		return -1;
	}
   img = imread( argv[1] );
   if (img.empty())
       return -1;

   namedWindow( "window", CV_WINDOW_NORMAL );
   namedWindow( "histogram", CV_WINDOW_NORMAL );
   setMouseCallback( "window", onMouse, 0 );

   imshow( "window", img );

   while((char)waitKey(100) != 27); // 27 = ascii value of ESC

   return 0;
}

enum drawingStatuses
{
	IDLE,
	DRAWING
}drawingStatus = IDLE;

double similarity()
{
	Mat area1, area2, roi;
	if(image_segments.size() != 2)
		return -1.0;

	// extract the first image
	vector<Point> points;
	points.push_back(image_segments.front().first);
	points.push_back(image_segments.front().second);
	roi = Mat(img, boundingRect(points));
	cvtColor(roi, area1, CV_BGR2HSV );
	// extract the second image
	points.clear();
	points.push_back(image_segments.back().first);
	points.push_back(image_segments.back().second);
	roi = Mat(img, boundingRect(points));
	cvtColor(roi, area2, CV_BGR2HSV );

	MatND hist_area1, hist_area2; //histograms

	/// Using 30 bins for hue and 32 for saturation
	int h_bins = 30; int s_bins = 32;
	int histSize[] = { h_bins, s_bins };

	// hue varies from 0 to 256, saturation from 0 to 180
	float h_ranges[] = { 0, 256 };
	float s_ranges[] = { 0, 180 };
	const float* ranges[] = { h_ranges, s_ranges };

	// Use the o-th and 1-st channels
	int channels[] = { 0, 1 };

	/// Calculate the histograms for the HSV images
	calcHist( &area1, 1, channels, Mat(), hist_area1, 2, histSize, ranges, true, false );
	normalize( hist_area1, hist_area1, 0, 1, NORM_MINMAX, -1, Mat() );

	calcHist( &area2, 1, channels, Mat(), hist_area2, 2, histSize, ranges, true, false );
	normalize( hist_area2, hist_area2, 0, 1, NORM_MINMAX, -1, Mat() );

	imshow("histogram", hist_area2);

	double correlation = compareHist( hist_area1, hist_area2, CV_COMP_CORREL );

	return correlation;
}

static void onMouse( int event, int x, int y, int, void* )
{
	if (event != CV_EVENT_LBUTTONDOWN && event != CV_EVENT_LBUTTONUP &&
			event != CV_EVENT_MOUSEMOVE)
	{
		return;
	}
	Mat result;
	img.copyTo(result); // a clean copy
	switch (drawingStatus) {
		case IDLE:
			if(event == CV_EVENT_LBUTTONDOWN)
			{
				drawingStatus = DRAWING;

				// start new rectangle
				current_segment.first.x = x;
				current_segment.first.y = y;

				//delete old rectangle
				if(image_segments.size() == 2)
					image_segments.pop_front();
			}
			break;
		case DRAWING:
			if(event == CV_EVENT_MOUSEMOVE)
			{
				rectangle(result, current_segment.first, Point(x,y), Scalar(100, 100, 255), 5);
			}
			else if (event == CV_EVENT_LBUTTONUP)
			{
				drawingStatus = IDLE;
				current_segment.second.x = x;
				current_segment.second.y = y;
				image_segments.push_back(current_segment);
			}
			break;
	}

	// draw rectangles
	for(rect_type::iterator r1 = image_segments.begin();
			r1 != image_segments.end(); ++r1)
	{
		rectangle(result, r1->first, r1->second, Scalar(100, 100, 255), 5);
	}

	// write correlation
	if (drawingStatus == IDLE && image_segments.size() == 2)
	{
		stringstream message;
		message << "Correlation: " << similarity();
		putText(result, message.str(), cvPoint(60,60),
				FONT_HERSHEY_COMPLEX_SMALL, 4.0, cvScalar(0,100,250), 5);
	}

	imshow( "window", result );
}
