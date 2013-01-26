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

#include "kulgur1/LandmarkMeasurement.h"
#include "kulgur1/LandmarkMeasurementArray.h"

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

      this->frontWheelVelocitiesSubscriber = this->node->subscribe<std_msgs::Float64MultiArray>("set_wheel_velocities", 
        50, &ROSModelPlugin::SetFrontWheelVelocities, this);

      this->rearWheelAnglesSubscriber = this->node->subscribe<std_msgs::Float64MultiArray>("set_rear_wheel_angles",
        50, &ROSModelPlugin::SetRearWheelAngles, this);

      this->wheelRotationsPublisher = this->node->advertise<std_msgs::Float64MultiArray>("wheel_rotations_and_angles", 50);

      this->landmarkMeasurementPublisher = this->node->advertise<LandmarkMeasurementArray>("visible_landmarks", 50);

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
        std_msgs::Float64MultiArray angles;

        angles.data.push_back(frontWheelJoints[0]->GetAngle(0).Radian());
        angles.data.push_back(frontWheelJoints[1]->GetAngle(0).Radian());

        angles.data.push_back(rearWheelJoints[0]->GetAngle(0).Radian());
        angles.data.push_back(rearWheelJoints[1]->GetAngle(0).Radian());

        wheelRotationsPublisher.publish(angles);

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

        measurementsMsg.timestamp.sec   = currentTime.sec;
        measurementsMsg.timestamp.nsec  = currentTime.nsec;

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
    
    // Max force applied to wheel joint, to reach desired velocity. Define robot's linear acceleration.
    static const double FrontWheelMaxForce      = 50.0;

    // Max force applied to rear wheel turning joints to reach desired turn path. Different axis than FrontWheelMaxForce!
    static const double RearWheelTurnMaxForce   = 50.0;

    // Max velocity that the rear wheel turning "servo" can achieve.
    static const double RearWheelTurnMaxVelocity= 5.0;

    // Wheel turn velocities will be controllerd with desiredVelocity = (currentAngle - desiredAngle)*ControlGain
    static const double RearWheelControlGain    = 10.0;

    static const double WheelRotationsPublishFreq = 50;

    //
    static const double LandmarkVisibilityPublishFreq = 10;

    // The field of view to left and right of robot where the landmark can be seen
    static const double LandmarkVisibilityFOV     = 1.047/2; // Same value is in model.sdf for camera

    // The max distance when Landmark can be seen
    static const double LandmarkVisibilityMaxDist = 50;

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

    ros::Publisher    wheelRotationsPublisher;
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

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ROSModelPlugin)
}
