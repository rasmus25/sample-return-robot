#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <ros/callback_queue.h>

namespace gazebo
{   
  class ROSModelPlugin : public ModelPlugin
  {

    public: ROSModelPlugin()
    {

      // Start up ROS
      std::string name = "ros_gazebo_diffdrive_wheel_velocities_plugin";
      int argc = 0;
      ros::init(argc, NULL, name);
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

      // This did not work. Probably something needs to be done with the message queue. From gazebo_ros_joint_trajectory.cpp example.
      // ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float64MultiArray>("set_wheel_velocities", 50, 
      //   boost::bind(&ROSModelPlugin::SetWheelVelocities, this, _1), ros::VoidPtr(), &this->queue_);
      // this->subscriber = this->node->subscribe(so);

      jointVelocities[0] = 0; jointVelocities[1] = 0;

      this->subscriber = this->node->subscribe<std_msgs::Float64MultiArray>("set_wheel_velocities", 50, &ROSModelPlugin::SetWheelVelocities, this);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateStart(
          boost::bind(&ROSModelPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      ros::spinOnce();

      this->model->GetJoint("left_wheel_hinge")->SetVelocity(0, jointVelocities[0]);
      this->model->GetJoint("right_wheel_hinge")->SetVelocity(0, jointVelocities[1]);

      this->model->GetJoint("left_wheel_hinge")->SetMaxForce(0, 3.0);
      this->model->GetJoint("right_wheel_hinge")->SetMaxForce(0, 3.0);

      // This works:
      // this->model->GetJoint("left_wheel_hinge")->SetForce(0, jointVelocities[0]);
      // this->model->GetJoint("right_wheel_hinge")->SetForce(0, jointVelocities[1]);
    }

    private: void SetWheelVelocities(const std_msgs::Float64MultiArray::ConstPtr& velocities)
    {
      jointVelocities[0] = velocities->data[0];
      jointVelocities[1] = velocities->data[1];

      // ROS_INFO("Setting wheel velocities: [%f %f]", velocities->data[0], velocities->data[1]);

      // this->model->GetJoint("left_wheel_hinge")->SetVelocity(0, velocities->data[0]);
      // this->model->GetJoint("right_wheel_hinge")->SetVelocity(0, velocities->data[1]);
    }
    
    double   jointVelocities[2];

    // Pointer to the model
    private: physics::ModelPtr model;

    //
    private: physics::WorldPtr world;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle* node;

    private: ros::Subscriber subscriber;

    private: ros::CallbackQueue queue_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ROSModelPlugin)
}
