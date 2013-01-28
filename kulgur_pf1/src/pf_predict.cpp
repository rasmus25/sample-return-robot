#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/ModelStates.h>

#include <mrpt/base.h>         // Include all classes in mrpt-base and its dependencies
#include <mrpt/gui.h>
 
using namespace mrpt;          // Global methods, and data types.
using namespace mrpt::utils;   // Select namespace for serialization, utilities, etc...
using namespace mrpt::poses;   // Select namespace for 2D & 3D geometry classes.
using namespace mrpt::gui;
using namespace Eigen;
using namespace std;

//
//
//

class KulgurOdometry
{
public:
	static const double WheelRadius;
	static const double WheelBase; 			// Distance of wheels from robot center axis
	static const double WheelAxisDist; 		// Distance between wheel axes
	static const double TurnWheelJointBase;	//

	static const double MinWheelTurnAngle;	// If wheel angles are smaller than this, then turn circle is not calculated and going straight.

	Vector3d PoseChange(const Vector4d& wheelRotations, const Vector2d rearWheelAngles)
	{
		if(!inited)
		{
			prevWheelRotations 	= wheelRotations;
			prevRearWheelAngles = rearWheelAngles;

			inited = true;

			return Vector3d(0, 0, 0);
		}

		Vector4d 	deltaWheels 	= wheelRotations - prevWheelRotations;							// Wheel rotation change from last call
		Vector2d	frontWheelDist	= Vector2d(deltaWheels[0], deltaWheels[1]) * WheelRadius;		// Distance travelled by the two front wheels

		double 		avgWheelDist  	= (frontWheelDist[0] + frontWheelDist[1])/2;

		// This is probably not the best way for calculating the azimuth change
		// double turnAngleByFrontWheels = (frontWheelDist[0] - frontWheelDist[1])/(2*WheelBase);

		double 		bearingChange = 0;
		double 		avgWheelTurnAng 	= (rearWheelAngles[0] + rearWheelAngles[1])/2;

		if(fabs(avgWheelTurnAng) >= MinWheelTurnAngle)
		{
			double avgWheelTurnRadius 	= WheelAxisDist / tan(avgWheelTurnAng);

			bearingChange 				= avgWheelDist / avgWheelTurnRadius;
		}

		Vector3d 	poseChange;

		// Approximating the travelled circle to line.
		poseChange[0] = cos(bearingChange/2) * avgWheelDist;
		poseChange[1] = sin(bearingChange/2) * avgWheelDist;
		poseChange[2] = bearingChange;

		prevWheelRotations 	= wheelRotations;
		prevRearWheelAngles = rearWheelAngles;

		return poseChange;
	}

	KulgurOdometry()
	{
		inited = false;
	}

private:
	bool 		inited;

	Vector4d 	prevWheelRotations;
	Vector2d 	prevRearWheelAngles;
};

const double KulgurOdometry::WheelRadius 		= 0.095;
const double KulgurOdometry::WheelBase 			= 0.2; 	// Distance of wheels from robot center axis
const double KulgurOdometry::WheelAxisDist 		= 0.48; // Distance between wheel axes
const double KulgurOdometry::TurnWheelJointBase = 0.2;	//

const double KulgurOdometry::MinWheelTurnAngle 	= 1e-6;	// If wheel angles are smaller than this, then turn circle is not calculated and going straight.

//
//
//

const int ParticleCount = 1;
typedef Matrix<double, 3, ParticleCount>  Particles;

void SetAllParticles(Particles* inout_particles, const Vector3d& state);

void PredictParticles(Particles* inout_particles, const Vector3d& poseChange);

void DrawParticles(const Particles& particles, CDisplayWindowPlots& plots);

void NewOdometryInfoCallback(const std_msgs::Float64MultiArray::ConstPtr& odometryInfo);
void NewPoseInfoCallback(const gazebo_msgs::ModelStates::ConstPtr& modelStates);

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

	ros::Subscriber subscriber = n.subscribe<std_msgs::Float64MultiArray>("/gazebo/wheel_rotations_and_angles", 50, &NewOdometryInfoCallback);

	ros::Subscriber modelPoseSubscriber = n.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 50, &NewPoseInfoCallback);

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

void NewOdometryInfoCallback(const std_msgs::Float64MultiArray::ConstPtr& odometryInfo)
{
	Vector4d wheelRotations(0, 0, 0, 0);
	Vector2d turnWheelAngles(0, 0);

	for(int i = 0 ; i < 2 ; i++) 	wheelRotations[i] = odometryInfo->data[i];
	for(int i = 0 ; i < 2 ; i++) 	turnWheelAngles[i] = odometryInfo->data[2 + i];

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

void NewPoseInfoCallback(const gazebo_msgs::ModelStates::ConstPtr& modelStates)
{
	lastPose = modelStates->pose[1];

	if(!inited)
	{
		double poseAngle = 2.0*acos(lastPose.orientation.w);	// NB! Valid only for our case when x=y=0
		SetAllParticles(&particles, Vector3d(lastPose.position.x, lastPose.position.y, poseAngle));
		inited = true;
	}
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