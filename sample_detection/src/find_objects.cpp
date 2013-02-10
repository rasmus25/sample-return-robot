#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/photo/photo.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>

using namespace cv;
using namespace std;

Mat img;

void spCallback(int,void*);
static void onMouse( int event, int x, int y, int, void* );
void findWhiteObjects();
void findHistogramPeak();

enum program_stages
{
	SHOW_IMAGE,
	FIND_WHITE_OBJECTS,
	FIND_HISTOGRAM_PEAK,
	ALL_DONE
}program_stage;

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
	setMouseCallback( "window", onMouse, 0 );
	imshow( "window", img );
	program_stage = SHOW_IMAGE;

	while((char)waitKey(100) != 27); // 27 = ascii value of ESC

	return 0;
}

static void onMouse( int event, int x, int y, int, void* )
{
	if(event == CV_EVENT_LBUTTONUP)
	{
		switch (program_stage) {
			case SHOW_IMAGE:
				program_stage = FIND_WHITE_OBJECTS;
				findWhiteObjects();
				break;
			case FIND_WHITE_OBJECTS:
				program_stage = FIND_HISTOGRAM_PEAK;
				findHistogramPeak();
				break;
			case FIND_HISTOGRAM_PEAK:
				program_stage = ALL_DONE;
				break;
			case ALL_DONE:
				program_stage = SHOW_IMAGE;
				imshow( "window", img );
				break;
			default:
				break;
		}
	}
}

void findWhiteObjects()
{
	Mat hsv, mask;
	cvtColor(img, hsv, CV_BGR2HSV);

	//OpenCV H=(0..180), S=(0..255), V=(0..255)
	uint8_t white_lower_value = 210;
	uint8_t white_upper_saturation = 30;

	//                   H  S  V
	inRange(hsv, Scalar(0, 0, white_lower_value), Scalar(180, white_upper_saturation, 255), mask);

	imshow( "window", mask );
}

void findHistogramPeak()
{
	Mat hsv, mask;
	cvtColor(img, hsv, CV_BGR2HSV);

	MatND big_histogram; //histograms

	/// 30x32 bins
	int h_bins = 30; int s_bins = 32;
	int histSize[] = { h_bins, s_bins };

	// hue varies from 0 to 179, saturation from 0 to 255
	float h_ranges[] = { 0, 180 };
	float s_ranges[] = { 0, 256 };
	const float* ranges[] = { h_ranges, s_ranges };

	// Use the o-th and 1-st channels
	int channels[] = { 0, 1 };

	/// Calculate the histograms for the HSV images
	calcHist( &hsv, 1, channels, Mat(), big_histogram, 2, histSize, ranges, true, false );
	normalize( big_histogram, big_histogram, 0, 1, NORM_MINMAX, -1, Mat() );

	int histogram_peak[2] = {0, 0}; //pure white color
	minMaxIdx( big_histogram, 0, 0, 0, histogram_peak, Mat() );

	// white is most prevalent color or no peak was found
	if (!histogram_peak[0] && !histogram_peak[1])
		return;

	// 10% of the peak value is a good threshold
	threshold(big_histogram, big_histogram, 0.1, 255, CV_THRESH_BINARY);

	// select only the biggest peak
	mask.create(big_histogram.rows+2, big_histogram.cols+2, CV_8UC1);
	mask = Scalar::all(0);
	floodFill(big_histogram, mask, Point(histogram_peak[0]+1,histogram_peak[1]+1), 255, 0, 0, 0,
						4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);

	imshow("window", mask);
}
