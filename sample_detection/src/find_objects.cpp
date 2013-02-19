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

Point2f projectToGround(Point2f pixel_coordinates);

void spCallback(int,void*);
static void onMouse( int event, int x, int y, int, void* );
void findWhiteObjects();
bool findHistogramPeak();
void filterPepper(const Mat &matrix);
void findCandidates(const Mat &matrix);
void ruleOutObvious();

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
			{
				program_stage = RULE_OUT_OBVIOUS;
				ruleOutObvious();
				Point2f world = projectToGround(Point(x,y));
				cout << world.x <<" "<< world.y << endl;
				break;
			}
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
	//TODO: not sure if need to clear
	contours.clear();
	hierarchy.clear();

	findContours( matrix, contours, hierarchy, CV_RETR_TREE,
			CV_CHAIN_APPROX_SIMPLE, Point(0,0) );

	/// Draw contours
	RNG rng(12345);
	Mat drawing;
	img.copyTo(drawing);

	for( int i = 0; i< contours.size(); i++ )
	{
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
	}

	/// Show in a window
	imshow("window", drawing);
}

// Where my task actually begins
void ruleOutObvious()
{
	//exclude small contours
	float min_width = 0.05, max_width=0.25;
	vector<bool> big_contours;
	Rect bounding;
	big_contours.reserve(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		bounding = boundingRect(contours[i]);
		Point2f lowerleft = Point2f(bounding.x, bounding.y+bounding.height);
		Point2f lowerright = Point2f(bounding.x+bounding.width, bounding.y+bounding.height);
		float width = projectToGround(lowerleft).y - projectToGround(lowerright).y;
		float distance = projectToGround(lowerleft).x;
		
		if(width > min_width && width < max_width && distance > 0 && distance < 30)
			big_contours.push_back(true);
		else
			big_contours.push_back(false);
	}
	/// Draw contours
	RNG rng(12345);
	Mat drawing;
	img.copyTo(drawing);

	for( int i = 0; i< contours.size(); i++ )
	{
		if(!big_contours[i])
			continue;
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
	}

	/// Show in a window
	imshow("window", drawing);
}

class cam
{
public:
	float pan;
	float tilt;
	float X;
	float Y;
	float Z; //height
	float fx;
	float fy;
	float cx; //principal point
	float cy;
	cam()
	{
		pan = 0.0; tilt = -0.32; // radians from front-pointing
		X = 0.0; Y = 0.0; Z = 0.55; //meters from robot origin
		fx = fy = 554.38; // from simulation
		cx = 320.5; cy = 240.5;
	}
}front_cam;

// how to project image pixel to ground plane in 7 easy steps:
// 1. Calibrate the camera and save the camera matrix and distortion matrix
// 2. Get resolution, camera matrix and distortion matrix from camera_info topic
// 3. Get camera rotation and translation matrices from a tf topic (write your own publisher for this)
// 4. Multiply the matrices to get a perspective transform
// 5. Invert the transformation matrix to get inverse transform
// 6. Calculate a mapping from image plane to ground plane
// 7. For every point, use the mapping to get projection on ground plane
//                       OR
// 1. Use closed-form formulas (Duda & Hart, 1973) and hard-coded values, for now
Point2f projectToGround(Point2f pixel_coordinates)
{
	Point2f onGround; // x is forward, y is left
	pixel_coordinates.x -= front_cam.cx;
	pixel_coordinates.y = front_cam.cy - pixel_coordinates.y;
	onGround.x = front_cam.Y - 
			(front_cam.Z*pixel_coordinates.x*sin(front_cam.pan)-
					(pixel_coordinates.y*sin(front_cam.tilt)-
					front_cam.fy*cos(front_cam.tilt))*
					front_cam.Z*cos(front_cam.pan))/
					(pixel_coordinates.y*cos(front_cam.tilt)+
							front_cam.fy*sin(front_cam.tilt));
	onGround.y = (front_cam.Z*pixel_coordinates.x*cos(front_cam.pan)+
						(pixel_coordinates.y*sin(front_cam.tilt)-
						front_cam.fy*cos(front_cam.tilt))*
						front_cam.Z*sin(front_cam.pan))/
						(pixel_coordinates.y*cos(front_cam.tilt)+
								front_cam.fy*sin(front_cam.tilt))-
								front_cam.X;
	return onGround;
}

