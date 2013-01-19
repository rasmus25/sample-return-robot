#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

namespace gazebo
{   
  class ROSModelPlugin : public ModelPlugin
  {

    public: ROSModelPlugin()
    {

      // Start up ROS
      std::string name = "ros_gazebo_diffdrive_wheel_angle_plugin";
      int argc = 0;
      ros::init(argc, NULL, name);

      publishingPeriod  = common::Time(1/50.0);
      lastPublishTime   = common::Time(0.0);
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

      // ROS Publisher
      pub = this->node->advertise<std_msgs::Float64MultiArray>("wheel_angles", 50);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateStart(
          boost::bind(&ROSModelPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      ros::spinOnce();

      common::Time currentTime =  this->world->GetSimTime();

      if(currentTime - lastPublishTime >= publishingPeriod)
      {
        std_msgs::Float64MultiArray angles;

        angles.data.push_back(this->model->GetJoint("left_wheel_hinge")->GetAngle(0).Radian());
        angles.data.push_back(this->model->GetJoint("right_wheel_hinge")->GetAngle(0).Radian());

        pub.publish(angles);

        lastPublishTime = currentTime;
      }
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    //
    private: physics::WorldPtr world;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle* node;

    // ROS Publisher
    private: ros::Publisher pub;

    private: common::Time publishingPeriod;
    private: common::Time lastPublishTime;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ROSModelPlugin)
}
