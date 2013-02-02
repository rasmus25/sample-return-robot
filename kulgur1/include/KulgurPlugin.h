#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <physics/Joint.hh>
#include <common/common.hh>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <ros/callback_queue.h>
#include <vector>

#include "kulgur_msgs/LandmarkMeasurementArray.h"
#include "kulgur_msgs/LowOdometry.h"

namespace gazebo
{   
	using namespace kulgur_msgs;
	using namespace std;

	class KulgurPlugin : public ModelPlugin
	{
	public:
		KulgurPlugin();

		void 	Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
		void 	OnUpdate();

	private:
		LandmarkMeasurement 	landmarkMeasurement(const math::Pose& sensorPoseInWorld, const math::Pose& landmarkPose);

		void 					setFrontWheelVelocities(const std_msgs::Float64MultiArray::ConstPtr& velocities);
		void 					setRearWheelAngles(const std_msgs::Float64MultiArray::ConstPtr& angles);

		void 					gazeboTimeToRosTime(const common::Time& gazeboTime, ros::Time* out_rosTime);
		void 					gazeboPoseToRosPose(const math::Pose& gazeboPose, geometry_msgs::Pose* out_rosPose);

		LowOdometry					odometryMsg();
		LandmarkMeasurementArray	landmarksMsg();

		LandmarkMeasurement 		landmarkToMeasurement(const math::Pose& sensorPoseInWorld, const math::Pose& landmarkPose);

	private:
	    double    desiredFrontWheelVelocities[2];
		double    desiredRearWheelAngles[2];

	    // This does not come from ROS topic, but is calculated here.
	    double    desiredRearWheelTurnVelocities[2];

	    // [0] - front_left_wheel, [1] - front_right_wheel
	    physics::JointPtr    frontWheelJoints[2];

	    // [0] - rear_left_wheel, [1] - rear_right_wheel
	    physics::JointPtr    rearWheelJoints[2];

	    ros::Subscriber   rearWheelAnglesSubscriber;
	    ros::Subscriber   frontWheelVelocitiesSubscriber;

	    ros::Publisher    odometryPublisher;
	    ros::Publisher    landmarkMeasurementPublisher;

	    common::Time odomPublishingPeriod;
	    common::Time odomLastPublishTime;

	    common::Time landmPublishingPeriod;
	    common::Time landmLastPublishTime;

	    //
	    std::vector<physics::ModelPtr>  landmarks;

	    physics::ModelPtr model;
	    physics::WorldPtr world;

	    // Pointer to the update event connection
	    event::ConnectionPtr updateConnection;
	    ros::NodeHandle* node;

	public:
		//
		// CONSTANTS
		//
		// Explained and defined in .cpp
	    static const double FrontWheelMaxForce;
	    static const double RearWheelTurnMaxForce;
	    static const double RearWheelTurnMaxVelocity;
	    static const double RearWheelControlGain;

	    static const double WheelRotationsPublishFreq;
	    static const double LandmarkVisibilityPublishFreq;

	    static const double LandmarkVisibilityFOV;
	    static const double LandmarkVisibilityMaxDist;

	    static const math::Pose LandmarksensorPoseInRobot;
	};

	GZ_REGISTER_MODEL_PLUGIN(KulgurPlugin)
}
