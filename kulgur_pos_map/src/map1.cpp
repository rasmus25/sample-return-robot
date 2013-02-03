#include <ros/ros.h>

#include <mrpt/base.h>         // Include all classes in mrpt-base and its dependencies
#include <mrpt/gui.h>

#include "kulgur_msgs/LandmarkMeasurementArray.h"
#include "kulgur_msgs/LowOdometry.h"

using namespace mrpt;          // Global methods, and data types.
using namespace mrpt::utils;   // Select namespace for serialization, utilities, etc...
using namespace mrpt::poses;   // Select namespace for 2D & 3D geometry classes.
using namespace mrpt::gui;
using namespace Eigen;
using namespace std;

using namespace kulgur_msgs;

//
// New message callbacks
//
void NewOdometryInfoCallback(const LowOdometry::ConstPtr& odometryInfo);
void NewLandmarkInfoCallback(const LandmarkMeasurementArray::ConstPtr& landmarkMeasurements);
//
//

// CDisplayWindow 		win;

CDisplayWindowPlots mapPlot("Particles");
CImage 				mapImage;


int main(int argc, char **argv)
{
	//
	ros::init(argc, argv, "PF1");
	ros::NodeHandle n;

	//
	// Load image
	//

	mapPlot.plot(vector<double>(1, 0), vector<double>(1, 0), "o10");
	mapPlot.axis(0, 1495, 0, 781);

	if(!mapImage.loadFromFile("map.bmp"))
	{
		ROS_INFO("Failed to load map.jpg !!!");
	}
	else
	{
		mapPlot.image(mapImage, 0, 0, 1495, 781);
		// win.showImage(mapImage);
	}

	//
	// Subscribe to topics
	//
	ros::Subscriber odoSubscriber = n.subscribe<LowOdometry>("/gazebo/kulgur1/odometry", 
		50, &NewOdometryInfoCallback);

	ros::Subscriber lmbSubscriber = n.subscribe<LandmarkMeasurementArray>("/gazebo/kulgur1/visible_landmarks", 
		2, &NewLandmarkInfoCallback);

	ros::spin();

	return 0;
}

void NewOdometryInfoCallback(const LowOdometry::ConstPtr& odometryInfo) {}
void NewLandmarkInfoCallback(const LandmarkMeasurementArray::ConstPtr& landmarkMeasurements) {}