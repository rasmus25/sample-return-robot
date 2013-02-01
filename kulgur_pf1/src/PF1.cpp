#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/ModelStates.h>

#include <mrpt/base.h>         // Include all classes in mrpt-base and its dependencies
#include <mrpt/gui.h>

#include "KulgurOdometry.h"
#include "PredictParticles.h"
#include "ParticleWeight.h"
#include "SamplerLowVariance.h"

#include "kulgur1/LandmarkMeasurementArray.h"
#include "kulgur1/LowOdometry.h"
 
using namespace mrpt;          // Global methods, and data types.
using namespace mrpt::utils;   // Select namespace for serialization, utilities, etc...
using namespace mrpt::poses;   // Select namespace for 2D & 3D geometry classes.
using namespace mrpt::gui;
using namespace Eigen;
using namespace std;

using namespace kulgur1;

//
// New message callbacks
//
void NewOdometryInfoCallback(const LowOdometry::ConstPtr& odometryInfo);
void NewLandmarkInfoCallback(const LandmarkMeasurementArray::ConstPtr& landmarkMeasurements);
//
//

void DrawParticles(const Particles& particles, CDisplayWindowPlots& plots);
void DrawLandmarks(const LandmarkMeasurementArray::ConstPtr& landmarkMeasurements, CDisplayWindowPlots& plots, 
	vector<bool> outliers = vector<bool>());


Particles 			particles;
geometry_msgs::Pose lastTruePose;
Vector3d			lastTruePoseVec;

Vector4d			prevWheelRotations;

CDisplayWindowPlots plots("Particles");

bool 				initedFromTruePose;

Matrix2d 			measurementCovariance;
LandmarksVector 	landmarks;

// This is dependent of measurementCovariance.
const double LambdaPhi = 0.0; //0001;

// Settings
const bool 	showPaths = false;

int main(int argc, char **argv)
{
	//
	initedFromTruePose = false;

	measurementCovariance << 0.1, 0, 0, 1.0;

	landmarks.push_back(Vector2d(30, 0));
	landmarks.push_back(Vector2d(0, 30));

	plots.axis(-5, 40, -5, 40);
	plots.resize(600, 600);

	//
	ros::init(argc, argv, "PF1");
	ros::NodeHandle n;

	ros::Subscriber odoSubscriber = n.subscribe<LowOdometry>("/gazebo/kulgur1/odometry", 
		50, &NewOdometryInfoCallback);

	ros::Subscriber lmbSubscriber = n.subscribe<LandmarkMeasurementArray>("/gazebo/kulgur1/visible_landmarks", 
		2, &NewLandmarkInfoCallback);

	plots.hold_on();
	ros::spin();

	return 0;
}

void NewOdometryInfoCallback(const LowOdometry::ConstPtr& odometryInfo)
{
	Vector4d wheelRotations;
	Vector2d turnWheelAngles;

	for(int i = 0 ; i < 2 ; i++) 	wheelRotations[i] = odometryInfo->front_wheel_rotations[i];
	for(int i = 0 ; i < 2 ; i++) 	turnWheelAngles[i] = odometryInfo->rear_wheel_angles[i];

	lastTruePose = odometryInfo->true_pose;
	
	double poseAngle = 2.0*acos(lastTruePose.orientation.w); // NB! Valid only for our case when x=y=0

	lastTruePoseVec = Vector3d(lastTruePose.position.x, lastTruePose.position.y, poseAngle);

	if(!initedFromTruePose)
	{
		SetAllParticles(&particles, lastTruePoseVec);

		prevWheelRotations = wheelRotations;
		initedFromTruePose = true;

		return;
	}

	PredictParticlesAddNoise(&particles, wheelRotations - prevWheelRotations, turnWheelAngles);

	// DrawParticles(particles, plots);

	//
	// if(!showPaths) plots.hold_on();
	// plots.plot(vector<double>(1, lastTruePose.position.x), vector<double>(1, lastTruePose.position.y), "b.3");
	// if(!showPaths) plots.hold_off();

	prevWheelRotations = wheelRotations;
}

void NewLandmarkInfoCallback(const LandmarkMeasurementArray::ConstPtr& landmarkMeasurements)
{
	//
	// Draw
	//
	plots.clear();
	DrawParticles(particles, plots);
	plots.plot(vector<double>(1, lastTruePose.position.x), vector<double>(1, lastTruePose.position.y), "b.3");
	//
	//

	if(landmarkMeasurements->measurements.size() == 0) return;

	// convert data
	MeasurementsVector measurementsVec;
	for(auto it = landmarkMeasurements->measurements.begin() ; it != landmarkMeasurements->measurements.end() ; it++)
	{
		measurementsVec.push_back(Vector2d(it->bearing, it->distance));
	}

	//
	vector<double> weights;
	vector<bool> outliers;
	ParticleWeights(measurementsVec, particles, landmarks, measurementCovariance, LambdaPhi, &weights, &outliers);

	vector<int> selectedParticles;
	SamplerLowVariance(weights, &selectedParticles);

	Particles newSet;
	int p = 0;

	for(auto it = selectedParticles.begin() ; it != selectedParticles.end() ; it++)
	{
		assert(*it < particles.cols());
		newSet.col(p) = particles.col(*it);
		p++;
	}

	particles = newSet;

	DrawLandmarks(landmarkMeasurements, plots, outliers);
}


void DrawParticles(const Particles& particles, CDisplayWindowPlots& plots)
{
	plots.plot(particles.row(0), particles.row(1), "g.3");
}

void DrawLandmarks(const LandmarkMeasurementArray::ConstPtr& landmarkMeasurements, CDisplayWindowPlots& plots, vector<bool> outliers)
{
	if(landmarkMeasurements->measurements.size() > 0)
	{
		for(int i = 0; i < landmarkMeasurements->measurements.size() ; i++)
		{
			const LandmarkMeasurement& m = landmarkMeasurements->measurements[i];

			// Euclidean vector to landmark in robot's frame.
			Vector2d mVec(m.distance * cos(m.bearing), m.distance * sin(m.bearing));

			// Landmark position in global frame
			Vector2d lm = Vector2d(lastTruePoseVec[0], lastTruePoseVec[1]) + Rotation2D<double>(lastTruePoseVec[2]) * mVec;

			if(outliers.empty() || !outliers[i])
			{
				plots.plot(vector<double>(1, lm[0]), vector<double>(1, lm[1]), "bo5");	
			}
			else
			{
				plots.plot(vector<double>(1, lm[0]), vector<double>(1, lm[1]), "ro5");
			}
		}
	}
}