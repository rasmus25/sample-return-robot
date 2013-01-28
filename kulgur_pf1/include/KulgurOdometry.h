#include <mrpt/base.h>         // Include all classes in mrpt-base and its dependencies
#include <mrpt/gui.h>

class KulgurOdometry
{
public:
	static const double WheelRadius;
	static const double WheelBase; 			// Distance of wheels from robot center axis
	static const double WheelAxisDist; 		// Distance between wheel axes
	static const double TurnWheelJointBase;	//

	static const double MinWheelTurnAngle;	// If wheel angles are smaller than this, then turn circle is not calculated and going straight.

	KulgurOdometry();
	Eigen::Vector3d PoseChange(const Eigen::Vector4d& wheelRotations, const Eigen::Vector2d rearWheelAngles) const;

	Eigen::Matrix<double, 3, 2>  PredictJacobian(const Eigen::Vector4d& wheelRotations, const Eigen::Vector2d& rearWheelAngles) const;

	Eigen::Vector3d PredictNoiseStdDev(const Eigen::Vector3d& poseChange) const;

private:
	double 			distanceTravelled(const Eigen::Vector4d& wheelRotations) const;
	double 			bearingChange(const Eigen::Vector4d& wheelRotations, const Eigen::Vector2d rearWheelAngles) const;
};
