#include "KulgurOdometry.h"

const double KulgurOdometry::WheelRadius 		= 0.095;
const double KulgurOdometry::WheelBase 			= 0.2; 	// Distance of wheels from robot center axis
const double KulgurOdometry::WheelAxisDist 		= 0.48; // Distance between wheel axes
const double KulgurOdometry::TurnWheelJointBase = 0.2;	//

const double KulgurOdometry::MinWheelTurnAngle 	= 1e-6;	// If wheel angles are smaller than this, then turn circle is not calculated and going straight.

using namespace Eigen;

Vector3d KulgurOdometry::PoseChange(const Vector4d& wheelRotations, const Vector2d rearWheelAngles)
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

KulgurOdometry::KulgurOdometry()
{
	inited = false;
}