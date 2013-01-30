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

#include "kulgur1/LandmarkMeasurementArray.h"
#include "kulgur1/LowOdometry.h"

namespace gazebo
{   
  using namespace kulgur1;
  using namespace std;

  class ROSModelPlugin : public ModelPlugin
  {

    public: ROSModelPlugin()
    {
      // Start up ROS
      std::string name = "ros_gazebo_kulgur1_plugin";
      int argc = 0;
      ros::init(argc, NULL, name);

      odomPublishingPeriod  = common::Time(1/WheelRotationsPublishFreq);
      odomLastPublishTime   = common::Time(0.0);

      landmPublishingPeriod = common::Time(1/LandmarkVisibilityPublishFreq);
      landmLastPublishTime  = common::Time(0.0);
    }

    public: ~ROSModelPlugin()
    {
      delete this->node;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->world = _parent->GetWorld();

      // ROS Nodehandle
      this->node = new ros::NodeHandle("~");

      this->frontWheelVelocitiesSubscriber = this->node->subscribe<std_msgs::Float64MultiArray>("kulgur1/set_wheel_velocities", 
        50, &ROSModelPlugin::SetFrontWheelVelocities, this);

      this->rearWheelAnglesSubscriber = this->node->subscribe<std_msgs::Float64MultiArray>("kulgur1/set_rear_wheel_angles",
        50, &ROSModelPlugin::SetRearWheelAngles, this);

      this->odometryPublisher = this->node->advertise<LowOdometry>("kulgur1/odometry", 50);

      this->landmarkMeasurementPublisher = this->node->advertise<LandmarkMeasurementArray>("kulgur1/visible_landmarks", 50);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateStart(
          boost::bind(&ROSModelPlugin::OnUpdate, this));

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
      landmarks.push_back(this->world->GetModel("pole1"));
      landmarks.push_back(this->world->GetModel("pole2"));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      ros::spinOnce();

      frontWheelJoints[0]->SetVelocity(0, desiredFrontWheelVelocities[0]);
      frontWheelJoints[1]->SetVelocity(0, desiredFrontWheelVelocities[1]);
      //
      //
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

      if(currentTime - odomLastPublishTime >= odomPublishingPeriod)
      {
        LowOdometry msg;

        msg.front_wheel_rotations[0] = frontWheelJoints[0]->GetAngle(0).Radian();
        msg.front_wheel_rotations[1] = frontWheelJoints[1]->GetAngle(0).Radian();

        msg.rear_wheel_angles[0] = rearWheelJoints[0]->GetAngle(0).Radian();
        msg.rear_wheel_angles[1] = rearWheelJoints[1]->GetAngle(0).Radian();

        GazeboPoseToRosPose(this->model->GetState().GetPose(), &msg.true_pose);

        GazeboTimeToRosTime(currentTime, &(msg.timestamp));

        odometryPublisher.publish(msg);

        odomLastPublishTime = currentTime;
      }

      //
      if (currentTime - landmLastPublishTime >= landmPublishingPeriod)
      {
        std::vector<LandmarkMeasurement> landmarkMeasurements;

        LandmarkMeasurementArray  measurementsMsg;

        for (std::vector<physics::ModelPtr>::iterator it = landmarks.begin(); it != landmarks.end(); ++it)
        {
          LandmarkMeasurement measurement = Landmark(this->model->GetState().GetPose() + LandmarksensorPoseInRobot, (*it)->GetState().GetPose());

          if (fabs(measurement.bearing) <= LandmarkVisibilityFOV && measurement.distance <= LandmarkVisibilityMaxDist)
          {
            measurementsMsg.measurements.push_back(measurement);
          }
        }

        GazeboTimeToRosTime(currentTime, &measurementsMsg.timestamp);

        landmarkMeasurementPublisher.publish(measurementsMsg);

        landmLastPublishTime = currentTime;
      }
    }

  private:
    LandmarkMeasurement Landmark(const math::Pose& sensorPoseInWorld, const math::Pose& landmarkPose)
    {
      math::Pose landmarkFromSensor = landmarkPose - sensorPoseInWorld;

      LandmarkMeasurement measurement;

      // Pole coordinates are defined at the middle. Currently height 10.
      measurement.distance  = (landmarkFromSensor.pos - math::Vector3(0, 0, 10)).GetLength();

      // Assuming no other rotation besides around z-axis
      measurement.bearing   = GZ_NORMALIZE(atan2(landmarkFromSensor.pos[1], landmarkFromSensor.pos[0]));

      // Assuming the landmark is straight.
      measurement.angle     = 0;

      return measurement;
    }

    void SetFrontWheelVelocities(const std_msgs::Float64MultiArray::ConstPtr& velocities)
    {
      desiredFrontWheelVelocities[0] = velocities->data[0];
      desiredFrontWheelVelocities[1] = velocities->data[1];
    }

    void SetRearWheelAngles(const std_msgs::Float64MultiArray::ConstPtr& angles)
    {
      desiredRearWheelAngles[0] = angles->data[0];
      desiredRearWheelAngles[1] = angles->data[1];
    }

    void GazeboTimeToRosTime(const common::Time& gazeboTime, ros::Time* out_rosTime)
    {
      *out_rosTime = ros::Time(gazeboTime.sec, gazeboTime.nsec);
    }

    void GazeboPoseToRosPose(const math::Pose& gazeboPose, geometry_msgs::Pose* out_rosPose)
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
    
    // Max force applied to wheel joint, to reach desired velocity. Define robot's linear acceleration.
    static const double FrontWheelMaxForce;

    // Max force applied to rear wheel turning joints to reach desired turn path. Different axis than FrontWheelMaxForce!
    static const double RearWheelTurnMaxForce;

    // Max velocity that the rear wheel turning "servo" can achieve.
    static const double RearWheelTurnMaxVelocity;

    // Wheel turn velocities will be controllerd with desiredVelocity = (currentAngle - desiredAngle)*ControlGain
    static const double RearWheelControlGain;

    static const double WheelRotationsPublishFreq;

    //
    static const double LandmarkVisibilityPublishFreq;

    // The field of view to left and right of robot where the landmark can be seen
    static const double LandmarkVisibilityFOV; // Same value is in model.sdf for camera

    // The max distance when Landmark can be seen
    static const double LandmarkVisibilityMaxDist;

    //
    static const math::Pose LandmarksensorPoseInRobot;

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

    private: physics::ModelPtr model;
    private: physics::WorldPtr world;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: ros::NodeHandle* node;

    // Why is this necessary?
    private: ros::CallbackQueue queue_;
  };

  // Landmark detector (camera?) is on height 0.3m from robot model coordinate center
  const math::Pose ROSModelPlugin::LandmarksensorPoseInRobot = math::Pose(math::Vector3(0, 0, 0.3), math::Quaternion(0, 0, 0));

    // Max force applied to wheel joint, to reach desired velocity. Define robot's linear acceleration.
    const double ROSModelPluginFrontWheelMaxForce      = 10.0;

    // Max force applied to rear wheel turning joints to reach desired turn path. Different axis than FrontWheelMaxForce!
    const double ROSModelPluginRearWheelTurnMaxForce   = 50.0;

    // Max velocity that the rear wheel turning "servo" can achieve.
    const double ROSModelPluginRearWheelTurnMaxVelocity= 5.0;

    // Wheel turn velocities will be controllerd with desiredVelocity = (currentAngle - desiredAngle)*ControlGain
    const double ROSModelPluginRearWheelControlGain    = 10.0;

    const double ROSModelPluginWheelRotationsPublishFreq = 50;

    //
    const double ROSModelPluginLandmarkVisibilityPublishFreq = 10;

    // The field of view to left and right of robot where the landmark can be seen
    const double ROSModelPluginLandmarkVisibilityFOV     = 1.047/2; // Same value is in model.sdf for camera

    // The max distance when Landmark can be seen
    const double ROSModelPluginLandmarkVisibilityMaxDist = 50;

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ROSModelPlugin)
}
