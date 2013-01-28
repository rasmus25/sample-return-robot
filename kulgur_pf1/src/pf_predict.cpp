#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/ModelStates.h>

#include <mrpt/base.h>         // Include all classes in mrpt-base and its dependencies
#include <mrpt/gui.h>

#include "KulgurOdometry.h"

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
//
//

const int ParticleCount = 1;
typedef Matrix<double, 3, ParticleCount>  Particles;

void SetAllParticles(Particles* inout_particles, const Vector3d& state);

void PredictParticles(Particles* inout_particles, const Vector3d& poseChange);

void DrawParticles(const Particles& particles, CDisplayWindowPlots& plots);

void NewOdometryInfoCallback(const LowOdometry::ConstPtr& odometryInfo);

//
//
//

Particles 			particles;
KulgurOdometry 		odometry;

CDisplayWindowPlots plots("Particles");

geometry_msgs::Pose lastPose;
bool 				inited;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pf_predict");
	ros::NodeHandle n;

	inited = false;

	ros::Subscriber subscriber = n.subscribe<LowOdometry>("/gazebo/kulgur1/odometry", 50, &NewOdometryInfoCallback);

	ros::Rate loopRate(10);

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

void PredictParticles(Particles* inout_particles, const Vector3d& poseChange)
{
	for (int i = 0; i < inout_particles->cols() ; ++i)
	{
		double currentAng = (*inout_particles)(2,i);

		Vector2d particlePoseChange 	= Rotation2D<double>(currentAng) * poseChange.block(0,0,2,1);

		inout_particles->block(0,i,2,1) += particlePoseChange;
		(*inout_particles)(2,i) 		+= poseChange[2];
	}
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
		inited = true;
	}

	Vector3d poseChange = odometry.PoseChange(wheelRotations, turnWheelAngles);
	PredictParticles(&particles, poseChange);

	DrawParticles(particles, plots);

	//
	plots.hold_on();
	plots.plot(vector<double>(1, lastPose.position.x), vector<double>(1, lastPose.position.y), "b.3");
	plots.hold_off();
}

//
//
//

void DrawParticles(const Particles& particles, CDisplayWindowPlots& plots)
{
	plots.plot(particles.row(0), particles.row(1), "g.3");
}

//
//
//

void SetAllParticles(Particles* inout_particles, const Vector3d& state)
{
	(*inout_particles) << 
							MatrixXd::Constant(1, inout_particles->cols(), state[0]) , 
							MatrixXd::Constant(1, inout_particles->cols(), state[1]) ,
							MatrixXd::Constant(1, inout_particles->cols(), state[2]); 
}