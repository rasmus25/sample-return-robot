#include "KulgurPlugin.h"

using namespace gazebo;
using namespace kulgur_msgs;
using namespace std;

KulgurPlugin::KulgurPlugin()
{
	string name = "Kulgur1Plugin";
	int argc = 0;
	ros::init(argc, NULL, name);

	odomPublishingPeriod  = common::Time(1/WheelRotationsPublishFreq);
	odomLastPublishTime   = common::Time(0.0);

	landmPublishingPeriod = common::Time(1/LandmarkVisibilityPublishFreq);
	landmLastPublishTime  = common::Time(0.0);
}

void KulgurPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
	// Store the pointer to the model
	this->model = _parent;
	this->world = _parent->GetWorld();

	// ROS Nodehandle
	this->node = new ros::NodeHandle("~");

	this->frontWheelVelocitiesSubscriber = this->node->subscribe<std_msgs::Float64MultiArray>("kulgur1/set_wheel_velocities", 
	50, &KulgurPlugin::setFrontWheelVelocities, this);

	this->rearWheelAnglesSubscriber = this->node->subscribe<std_msgs::Float64MultiArray>("kulgur1/set_rear_wheel_angles",
	50, &KulgurPlugin::setRearWheelAngles, this);

	this->odometryPublisher = this->node->advertise<LowOdometry>("kulgur1/odometry", 50);

	this->landmarkMeasurementPublisher = this->node->advertise<LandmarkMeasurementArray>("kulgur1/visible_landmarks", 2);

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateStart(
	  boost::bind(&KulgurPlugin::OnUpdate, this));

	//
	frontWheelJoints[0] = this->model->GetJoint("front_left_wheel_hinge");
	frontWheelJoints[1] = this->model->GetJoint("front_right_wheel_hinge");

	frontWheelJoints[0]->SetMaxForce(0, FrontWheelMaxForce);
	frontWheelJoints[1]->SetMaxForce(0, FrontWheelMaxForce);

	desiredFrontWheelVelocities[0] = 0;
	desiredFrontWheelVelocities[1] = 0;

	rearWheelJoints[0] = this->model->GetJoint("rear_left_wheel_hinge");
	rearWheelJoints[1] = this->model->GetJoint("rear_right_wheel_hinge");

	rearWheelJoints[0]->SetMaxForce(0, RearWheelTurnMaxForce);
	rearWheelJoints[1]->SetMaxForce(0, RearWheelTurnMaxForce);

	//
	// landmarks.push_back(this->world->GetModel("pole1"));
	// landmarks.push_back(this->world->GetModel("pole2"));

	landmarks.push_back(this->world->GetModel("pole600"));
	landmarks.push_back(this->world->GetModel("pole601"));
	landmarks.push_back(this->world->GetModel("pole602"));
	landmarks.push_back(this->world->GetModel("pole603"));
}

void KulgurPlugin::OnUpdate()
{
	ros::spinOnce();

	//
	// Set front wheel velocities. 
	//
	frontWheelJoints[0]->SetVelocity(0, desiredFrontWheelVelocities[0]);
	frontWheelJoints[1]->SetVelocity(0, desiredFrontWheelVelocities[1]);
	
	//
	// Turn rear wheels
	//
	for(int i = 0 ; i < 2 ; i++)
	{
		desiredRearWheelTurnVelocities[i] = RearWheelControlGain * 
			(desiredRearWheelAngles[i] - rearWheelJoints[i]->GetAngle(0).Radian());

		// TODO: Add velocity limit.
		rearWheelJoints[i]->SetVelocity(0, desiredRearWheelTurnVelocities[i]);
	}

	//  
	//
	//
	common::Time currentTime =  this->world->GetSimTime();

	//
	// Odometry message
	//
	if(currentTime - odomLastPublishTime >= odomPublishingPeriod)
	{
		LowOdometry msg = odometryMsg();

		gazeboTimeToRosTime(currentTime, &(msg.timestamp));
		odometryPublisher.publish(msg);

		odomLastPublishTime = currentTime;
	}

	//
	// Landmarks message
	//
	if (currentTime - landmLastPublishTime >= landmPublishingPeriod)
	{
		LandmarkMeasurementArray  measurementsMsg = landmarksMsg();

		gazeboTimeToRosTime(currentTime, &measurementsMsg.timestamp);
		landmarkMeasurementPublisher.publish(measurementsMsg);

		landmLastPublishTime = currentTime;
	}
}

LowOdometry KulgurPlugin::odometryMsg()
{
	LowOdometry msg;

	msg.front_wheel_rotations[0] = frontWheelJoints[0]->GetAngle(0).Radian();
	msg.front_wheel_rotations[1] = frontWheelJoints[1]->GetAngle(0).Radian();

	msg.rear_wheel_angles[0] = rearWheelJoints[0]->GetAngle(0).Radian();
	msg.rear_wheel_angles[1] = rearWheelJoints[1]->GetAngle(0).Radian();

	gazeboPoseToRosPose(this->model->GetState().GetPose(), &msg.true_pose);

	return msg;
}

LandmarkMeasurementArray KulgurPlugin::landmarksMsg()
{
	LandmarkMeasurementArray  msg;

	for (std::vector<physics::ModelPtr>::iterator it = landmarks.begin(); it != landmarks.end(); ++it)
	{
		LandmarkMeasurement measurement = landmarkToMeasurement(this->model->GetState().GetPose() + LandmarksensorPoseInRobot, (*it)->GetState().GetPose());

		if (fabs(measurement.bearing) <= LandmarkVisibilityFOV && measurement.distance <= LandmarkVisibilityMaxDist)
		{
			msg.measurements.push_back(measurement);
		}
	}

	return msg;
}

LandmarkMeasurement KulgurPlugin::landmarkToMeasurement(const math::Pose& sensorPoseInWorld, const math::Pose& landmarkPose)
{
	math::Pose landmarkFromSensor = landmarkPose - sensorPoseInWorld;

	LandmarkMeasurement measurement;

	// Pole coordinates are defined at the middle. Currently height 10.
	measurement.distance  = math::Vector2d(landmarkPose.pos.x, landmarkPose.pos.y).Distance(
	math::Vector2d(sensorPoseInWorld.pos.x, sensorPoseInWorld.pos.y));

	// Assuming no other rotation besides around z-axis
	// measurement.bearing   = GZ_NORMALIZE(atan2(landmarkFromSensor.pos[1], landmarkFromSensor.pos[0]));
	measurement.bearing = GZ_NORMALIZE(
	atan2(landmarkPose.pos.y - sensorPoseInWorld.pos.y, landmarkPose.pos.x 
	        - sensorPoseInWorld.pos.x) - 2.0*acos(sensorPoseInWorld.rot.w));

	// Assuming the landmark is straight.
	measurement.angle    = 0;

	return measurement;	
}

void KulgurPlugin::setFrontWheelVelocities(const std_msgs::Float64MultiArray::ConstPtr& velocities)
{
	desiredFrontWheelVelocities[0] = velocities->data[0];
	desiredFrontWheelVelocities[1] = velocities->data[1];
}

void KulgurPlugin::setRearWheelAngles(const std_msgs::Float64MultiArray::ConstPtr& angles)
{
	desiredRearWheelAngles[0] = angles->data[0];
	desiredRearWheelAngles[1] = angles->data[1];
}

void KulgurPlugin::gazeboTimeToRosTime(const common::Time& gazeboTime, ros::Time* out_rosTime)
{
	*out_rosTime = ros::Time(gazeboTime.sec, gazeboTime.nsec);
}

void KulgurPlugin::gazeboPoseToRosPose(const math::Pose& gazeboPose, geometry_msgs::Pose* out_rosPose)
{
	geometry_msgs::Point point;

	point.x = gazeboPose.pos.x;
	point.y = gazeboPose.pos.y;
	point.z = gazeboPose.pos.z;

	geometry_msgs::Quaternion rot;

	rot.x = gazeboPose.rot.x;
	rot.y = gazeboPose.rot.y;
	rot.z = gazeboPose.rot.z;
	rot.w = gazeboPose.rot.w;

	out_rosPose->position = point;
	out_rosPose->orientation = rot;
}


// Landmark detector (camera?) is on height 0.3m from robot model coordinate center
const math::Pose KulgurPlugin::LandmarksensorPoseInRobot = math::Pose(math::Vector3(0, 0, 0.3), math::Quaternion(0, 0, 0));

// Max force applied to wheel joint, to reach desired velocity. Define robot's linear acceleration.
const double KulgurPlugin::FrontWheelMaxForce      = 10.0;

// Max force applied to rear wheel turning joints to reach desired turn path. Different axis than FrontWheelMaxForce!
const double KulgurPlugin::RearWheelTurnMaxForce   = 50.0;

// Max velocity that the rear wheel turning "servo" can achieve.
const double KulgurPlugin::RearWheelTurnMaxVelocity= 5.0;

// Wheel turn velocities will be controllerd with desiredVelocity = (currentAngle - desiredAngle)*ControlGain
const double KulgurPlugin::RearWheelControlGain    = 10.0;

//
const double KulgurPlugin::WheelRotationsPublishFreq = 10;

//
const double KulgurPlugin::LandmarkVisibilityPublishFreq = 10;

// The field of view to left and right of robot where the landmark can be seen
const double KulgurPlugin::LandmarkVisibilityFOV     = 1.047/2; // Same value is in model.sdf for camera

// The max distance when Landmark can be seen
const double KulgurPlugin::LandmarkVisibilityMaxDist = 50;