#include <random>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/ModelStates.h>

#include <mrpt/base.h>         // Include all classes in mrpt-base and its dependencies
#include <mrpt/gui.h>

#include "KulgurOdometry.h"
#include "Particles.h"
#include "PredictParticles.h"

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
//
//

void DrawParticles(const Particles& particles, CDisplayWindowPlots& plots);

void NewOdometryInfoCallback(const LowOdometry::ConstPtr& odometryInfo);

//
//
//

Particles 			particles;
KulgurOdometry 		odometry;

Vector4d			prevWheelRotations;

CDisplayWindowPlots plots("Particles");

geometry_msgs::Pose lastPose;
bool 				inited;

const bool  		showPaths = true;
const bool 			AddNoise_Conf = true;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pf_predict");
	ros::NodeHandle n;

	inited = false;

	ros::Subscriber subscriber = n.subscribe<LowOdometry>("/gazebo/kulgur_msgs/odometry", 50, &NewOdometryInfoCallback);

	ros::Rate loopRate(10);

	if(showPaths) plots.hold_on();

	ros::spin();

	// while(ros::ok() && plots.isOpen())
	// {
	// 	DrawParticles(particles, plots);
	// 	loopRate.sleep();
	// }

	return 0;
}

//
//
//

void NewOdometryInfoCallback(const LowOdometry::ConstPtr& odometryInfo)
{
	Vector4d wheelRotations(0, 0, 0, 0);
	Vector2d turnWheelAngles(0, 0);

	for(int i = 0 ; i < 2 ; i++) 	wheelRotations[i] = odometryInfo->front_wheel_rotations[i];
	for(int i = 0 ; i < 2 ; i++) 	turnWheelAngles[i] = odometryInfo->rear_wheel_angles[i];

	lastPose = odometryInfo->true_pose;

	if(!inited)
	{
		double poseAngle = 2.0*acos(lastPose.orientation.w);	// NB! Valid only for our case when x=y=0
		SetAllParticles(&particles, Vector3d(lastPose.position.x, lastPose.position.y, poseAngle));
		prevWheelRotations = wheelRotations;
		inited = true;

		return;
	}

	PredictParticlesAddNoise(&particles, wheelRotations - prevWheelRotations, turnWheelAngles);

	DrawParticles(particles, plots);

	//
	if(!showPaths) plots.hold_on();
	plots.plot(vector<double>(1, lastPose.position.x), vector<double>(1, lastPose.position.y), "b.1");
	if(!showPaths) plots.hold_off();

	prevWheelRotations = wheelRotations;
}

//
//
//

void DrawParticles(const Particles& particles, CDisplayWindowPlots& plots)
{
	plots.plot(particles.row(0), particles.row(1), "g.1");
}

//
//
//


