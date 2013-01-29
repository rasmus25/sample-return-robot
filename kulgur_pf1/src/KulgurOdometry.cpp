#include "KulgurOdometry.h"

#include <cmath>

const double KulgurOdometry::WheelRadius 		= 0.095;
const double KulgurOdometry::WheelBase 			= 0.2; 	// Distance of wheels from robot center axis
const double KulgurOdometry::WheelAxisDist 		= 0.48; // Distance between wheel axes
const double KulgurOdometry::TurnWheelJointBase = 0.2;	//

const double KulgurOdometry::MinWheelTurnAngle 	= 1e-6;	// If wheel angles are smaller than this, then turn circle is not calculated and going straight.

using namespace Eigen;

Eigen::Matrix<double, 3, 2>  KulgurOdometry::PredictJacobian(const Vector4d& wheelRotations, const Vector2d& rearWheelAngles) const
{
	double beta = (rearWheelAngles[0] + rearWheelAngles[1])/2;			// Average angle of turning wheels
	double dPhi = bearingChange(wheelRotations, rearWheelAngles);		// Change of robot's bearing
	double Da 	= distanceTravelled(wheelRotations);					// Average circular section distance travelled by wheels

	double D_dPhi__D_beta 	= pow(1/cos(beta), 2) * Da / WheelAxisDist;
	double D_dPhi__D_Da		= tan(beta) / WheelAxisDist;

	double D_x__D_Da 	= -sin(dPhi/2) * D_dPhi__D_Da + cos(dPhi/2);
	double D_y__D_Da 	= cos(dPhi/2) * D_dPhi__D_Da + sin(dPhi/2);

	double D_x__D_beta 		= -sin(dPhi/2) * D_dPhi__D_beta;
	double D_y__D_beta		= cos(dPhi/2) * D_dPhi__D_beta;

	Matrix<double, 3, 2> jac;

	jac << 	D_x__D_Da, D_x__D_beta,
			D_y__D_Da, D_y__D_beta,
			D_dPhi__D_Da, D_dPhi__D_beta;

	return jac;
}

//
// Uses average wheel rotations for travelled distance and average rearWheelAngles
// for calculating turning radius accordig to 
// TurningRadius = (Distance of wheel axes) / tan (average angle of turning wheels)
// The travelled segment of the circle is approximated by a line.
// Output is change of robot pose [x, y, angle] in robot's coordinate frame.
//
Vector3d KulgurOdometry::PoseChange(const Vector4d& wheelRotations, const Vector2d rearWheelAngles) const
{
	double 		Da 	 = distanceTravelled(wheelRotations);					// Distance travelled averaged for two wheels
	double 		dPhi = bearingChange(wheelRotations, rearWheelAngles);		// Bearing change


	Vector3d 	poseChange;

	// Approximating the travelled circle to line.
	poseChange[0] = cos(dPhi/2) * Da;
	poseChange[1] = sin(dPhi/2) * Da;
	poseChange[2] = dPhi;

	return poseChange;
}

double KulgurOdometry::distanceTravelled(const Eigen::Vector4d& wheelRotations) const
{
	Vector2d	frontWheelDist	= Vector2d(wheelRotations[0], wheelRotations[1]) * WheelRadius;		// Distance travelled by the two front wheels

	return (frontWheelDist[0] + frontWheelDist[1])/2;
}

double KulgurOdometry::bearingChange(const Eigen::Vector4d& wheelRotations, const Eigen::Vector2d rearWheelAngles) const
{
	double		bearingChange 		= 0;
	double 		avgWheelTurnAng 	= (rearWheelAngles[0] + rearWheelAngles[1])/2;
	double 		avgWheelDist 		= distanceTravelled(wheelRotations);

	if(fabs(avgWheelTurnAng) >= MinWheelTurnAngle)
	{

		// This did not give a better result.
		// double avgWheelTurnRadius 	= WheelAxisDist / (tan(rearWheelAngles[0])/2 + tan(rearWheelAngles[1])/2) ;

		double avgWheelTurnRadius 	= WheelAxisDist / tan(avgWheelTurnAng);

		bearingChange 				= avgWheelDist / avgWheelTurnRadius;
	}

	return bearingChange;
}

KulgurOdometry::KulgurOdometry()
{
}