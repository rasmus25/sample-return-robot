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

Mat img, grass_mask;

vector<vector<Point> > contours;
vector<Vec4i> hierarchy;

void spCallback(int,void*);
static void onMouse( int event, int x, int y, int, void* );
void findWhiteObjects();
bool findHistogramPeak();
void filterPepper(const Mat &matrix);
void findCandidates(const Mat &matrix);

enum program_stages
{
	SHOW_IMAGE,
	FIND_WHITE_OBJECTS,
	FIND_HISTOGRAM_PEAK,
	FILTER_NOISE,
	FIND_CANDIDATES,
	RULE_OUT_OBVIOUS,
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
	namedWindow( "histogram", CV_WINDOW_NORMAL );
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
				if(!findHistogramPeak())
					program_stage = ALL_DONE;
				break;
			case FIND_HISTOGRAM_PEAK:
				program_stage = FILTER_NOISE;
				//Filter out small black specks
				filterPepper(grass_mask);
				break;
			case FILTER_NOISE:
				program_stage = FIND_CANDIDATES;
				findCandidates(grass_mask);
				break;
			case FIND_CANDIDATES:
				program_stage = RULE_OUT_OBVIOUS;
				break;
			case RULE_OUT_OBVIOUS:
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

void filterPepper(const Mat &matrix)
{
	int erosion_size = 3;
	Mat element = getStructuringElement( MORPH_RECT,
									   Size( 2*erosion_size + 1, 2*erosion_size+1 ),
									   Point( erosion_size, erosion_size ) );
	dilate( matrix, matrix, element );
	erode( matrix, matrix, element );
	imshow( "window", matrix );
}


bool findHistogramPeak()
{
	Mat hsv, mask, lookup_table;
	cvtColor(img, hsv, CV_BGR2HSV);

	Mat big_histogram; //histograms

	/// 30x32 bins
	int h_bins = 30; int s_bins = 32;
	int histSize[] = { h_bins, s_bins };

	// hue varies from 0 to 179, saturation from 0 to 255
	float h_ranges[] = { 0, 180 };
	float s_ranges[] = { 0, 256 };
	const float* ranges[] = { h_ranges, s_ranges };

	// Use the 0-th and 1-st channels
	int channels[] = { 0, 1 };

	/// Calculate the histograms for the HSV images
	calcHist( &hsv, 1, channels, Mat(), big_histogram, 2, histSize, ranges, true, false );

	// ignore white color peak (useful during testing in the winter)
	big_histogram.at<double>(0,0) = 0;
	normalize( big_histogram, big_histogram, 0, 255, NORM_MINMAX, -1, Mat() );

	// find the peak value on the 2D histogram
	int histogram_peak[2] = {0, 0}; //pure white color
	minMaxIdx( big_histogram, 0, 0, 0, histogram_peak, Mat() );

	// if white is most prevalent color or no peak was found
	if (histogram_peak[0]<=0 || histogram_peak[0]>=h_bins || histogram_peak[1]<=0 || histogram_peak[1]>=s_bins)
		return false;

	// TODO: 10% of the peak value is a good threshold, i guess, but have to test
	threshold(big_histogram, big_histogram, 25, 255, CV_THRESH_BINARY);

	// select only the biggest peak, if there are several over threshold
	// others are masked out
	mask.create(big_histogram.rows+2, big_histogram.cols+2, CV_8UC1);
	mask = Scalar::all(0);
	big_histogram.at<double>(histogram_peak[1],histogram_peak[0]) = 0;
	floodFill(big_histogram, mask, Point(histogram_peak[1],histogram_peak[0]), 255, 0, 0, 0,
						4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);

	// floodfill's mask is the new histogram
	// remove extra columns and rows to get a correctly sized histogram
	Rect without_edges(1, 1, big_histogram.cols, big_histogram.rows);
	lookup_table = mask(without_edges);
	lookup_table.convertTo(lookup_table, CV_32F);

	/// Project the histogram peak back to the original image to find the most prevalent color
	// TIP: if there is only one color peak then backprojecting the histogram of an image
	//      to the image itself does a good job of highlighting the color. In case of several
	//      peaks, we need to select one, as is done above.
	Mat backproj;
	calcBackProject( &hsv, 1, channels, lookup_table, backproj, ranges, 1, true );

	// output to global variable
	grass_mask = backproj;

	imshow("window", backproj);
	imshow("histogram", big_histogram);
	return true;
}
void findCandidates(const Mat &matrix)
{
	// TODO: find another starting point
	findContours( matrix, contours, hierarchy, CV_RETR_TREE,
			CV_CHAIN_APPROX_SIMPLE, Point(matrix.size().width/2, matrix.size().height-2) );

	//exclude small contours
	double min_area = 200, max_area=200000;
	vector<bool> big_contours;
	big_contours.reserve(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		if(contourArea(contours[i]) > min_area && contourArea(contours[i]) < max_area)
			big_contours.push_back(true);
		else
			big_contours.push_back(false);
	}
	/// Draw contours
	RNG rng(12345);
//			Mat drawing = Mat::zeros( thresholded_image.size(), CV_8UC3 );
	Mat drawing;
	img.copyTo(drawing);
	cout << "test ";
	cout << contours.size();
	cout << " end" << endl;

	Rect bounding;

	for( int i = 0; i< contours.size(); i++ )
	{
		if(!big_contours[i])
			continue;
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

		bounding = boundingRect( Mat(contours[i]) );
		cout << bounding.x <<" "<< bounding.y <<" "<< bounding.width <<" "<< bounding.height << endl;
		rectangle( drawing, bounding.tl(), bounding.br(), color, 2, 8, 0 );

		drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
	}

	// FIXME: does not draw stuff
	rectangle( drawing, Point(bounding.x, bounding.y), Point(bounding.x+10, bounding.y+10), Scalar(0,100,250), -1, 8, 0 );
	rectangle( drawing, Point( 0, 100 ), Point( 100, 200), Scalar( 0, 255, 255 ), 2, 8, 0);
	string message = "test";
	putText(drawing, message, cvPoint(60,60),
			FONT_HERSHEY_COMPLEX_SMALL, 4.0, cvScalar(0,100,250), 5);
	/// Show in a window
	imshow("window", drawing);
}

