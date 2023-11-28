#ifndef _JOINT_VEL_CONTROLLER_PLUGIN_HH_
#define _JOINT_VEL_CONTROLLER_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class JointVelControllerPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: JointVelControllerPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // // Safety check
      // if (_model->GetJointCount() == 0)
      // {
      //   std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
      //   return;
      // }

      // Store the model pointer for convenience.
      this->model = _model;

      // Get joint's namespace
      this->joint_namespace = this->model->GetName ();
      if ( !_sdf->HasElement ( "robotNamespace" ) ) {
          ROS_INFO_NAMED(this->joint_namespace, "GazeboRosJointVelController Plugin missing <robotNamespace>, defaults to \"%s\"",
                    this->joint_namespace.c_str() );
      } else {
          this->joint_namespace = _sdf->GetElement ( "robotNamespace" )->Get<std::string>();
          if ( this->joint_namespace.empty() ) this->joint_namespace = this->model->GetName ();
      }
      if ( !joint_namespace.empty() ) this->joint_namespace += "/";

      // Get joint
      std::string joint_name;
      if ( !_sdf->HasElement ( "jointName" ) ) {
          ROS_ASSERT ( "GazeboRosJointVelController Plugin missing jointNames" );
      } else {
          sdf::ElementPtr element = _sdf->GetElement ( "jointName" ) ;
          joint_name = element->Get<std::string>();
      }
      this->joint = this->model->GetJoint(joint_name);
      if (!this->joint)
        ROS_FATAL_NAMED(this->joint_namespace, "Joint %s does not exist!", joint_name.c_str());

      // Setup a P-controller, with a gain of 0.1.
      this->pid = common::PID(0.1, 0, 0);

      if (_sdf->HasElement("p_gain"))
      {
        double p_gain = _sdf->Get<double>("p_gain");
        this->pid.SetPGain(p_gain);
        ROS_INFO_NAMED(this->joint_namespace, "Set p_gain %lf", p_gain);
      }
      if (_sdf->HasElement("i_gain"))
      {
        double i_gain = _sdf->Get<double>("i_gain");
        this->pid.SetIGain(i_gain);
        ROS_INFO_NAMED(this->joint_namespace, "Set i_gain %lf", i_gain);
      }
      if (_sdf->HasElement("d_gain"))
      {
        double d_gain = _sdf->Get<double>("d_gain");
        this->pid.SetDGain(d_gain);
        ROS_INFO_NAMED(this->joint_namespace, "Set d_gain %lf", d_gain);
      }
      if (_sdf->HasElement("i_min"))
      {
        double i_min = _sdf->Get<double>("i_min");
        this->pid.SetIMin(i_min);
        ROS_INFO_NAMED(this->joint_namespace, "Set i_min %lf", i_min);
      }
      if (_sdf->HasElement("i_max"))
      {
        double i_max = _sdf->Get<double>("i_max");
        this->pid.SetIMax(i_max);
        ROS_INFO_NAMED(this->joint_namespace, "Set i_max %lf", i_max);
      }
      if (_sdf->HasElement("cmd_min"))
      {
        double cmd_min = _sdf->Get<double>("cmd_min");
        this->pid.SetCmdMin(cmd_min);
        ROS_INFO_NAMED(this->joint_namespace, "Set cmd_min %lf", cmd_min);
      }
      if (_sdf->HasElement("cmd_max"))
      {
        double cmd_max = _sdf->Get<double>("cmd_max");
        this->pid.SetCmdMax(cmd_max);
        ROS_INFO_NAMED(this->joint_namespace, "Set cmd_max %lf", cmd_max);
      }

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(
          this->joint->GetScopedName(), this->pid);

      // Default to zero velocity
      double velocity = 0;

      // Check that the velocity element exists, then read the value
      if (_sdf->HasElement("velocity"))
        velocity = _sdf->Get<double>("velocity");

      this->SetVelocity(velocity);

      // // Create the node
      // this->node = transport::NodePtr(new transport::Node());
      // #if GAZEBO_MAJOR_VERSION < 8
      // this->node->Init(this->model->GetWorld()->GetName());
      // #else
      // this->node->Init(this->model->GetWorld()->Name());
      // #endif

      // // Create a topic name
      // std::string topicName = "~/" + this->joint_namespace + "/vel_cmd";

      // // Subscribe to the topic, and register a callback
      // this->sub = this->node->Subscribe(topicName,
      //    &JointVelControllerPlugin::OnMsg, this);

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle(this->joint_namespace));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->joint_namespace + "/vel_cmd",
            1,
            boost::bind(&JointVelControllerPlugin::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&JointVelControllerPlugin::QueueThread, this));

    }

    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
    public: void SetVelocity(const double &_vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint->GetScopedName(), _vel);
    }

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnMsg(ConstVector3dPtr &_msg)
    {
      this->SetVelocity(_msg->x());
    }

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
    public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetVelocity(_msg->data);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    // /// \brief A node used for transport
    // private: transport::NodePtr node;

    // /// \brief A subscriber to a named topic.
    // private: transport::SubscriberPtr sub;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    private: std::string joint_namespace;

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(JointVelControllerPlugin)
}
#endif