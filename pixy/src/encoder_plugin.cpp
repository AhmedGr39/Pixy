#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <robot_msgs/EncoderData.h>

namespace gazebo
{
  class EncoderPlugin : public ModelPlugin
  {
    public:
      EncoderPlugin() {}

      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        // Get the joint to which the encoder is attached
        std::string joint_name = _sdf->Get<std::string>("joint_name");
        this->joint = _model->GetJoint(joint_name);
        if (!this->joint)
        {
          ROS_ERROR_STREAM("Joint '" << joint_name << "' not found for encoder plugin!");
          return;
        }

        // Get the ROS node handle
        if (!ros::isInitialized())
        {
          ROS_FATAL_STREAM("ROS node is not initialized!");
          return;
        }
        this->node = new ros::NodeHandle("");

        // Create a ROS publisher for the encoder data
        std::string topic_name = _sdf->Get<std::string>("topic_name", "encoder_data");
        this->pub = this->node->advertise<robot_msgs::EncoderData>(topic_name, 1);

        // Connect to the Gazebo update event
        this->update_connection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&EncoderPlugin::OnUpdate, this, _1));
      }

      virtual void Reset() {}

    private:
      void OnUpdate(const common::UpdateInfo& _info)
      {
        // Get the joint angle and velocity
        double angle = this->joint->GetAngle(0).Radian();
        double velocity = this->joint->GetVelocity(0);

        // Publish the encoder data
        robot_msgs::EncoderData msg;
        msg.angle = angle;
        msg.velocity = velocity;
        this->pub.publish(msg);
      }

      physics::JointPtr joint;
      ros::NodeHandle* node;
      ros::Publisher pub;
      event::ConnectionPtr update_connection;
  };

  GZ_REGISTER_MODEL_PLUGIN(EncoderPlugin)

  class LeftEncoderPlugin : public EncoderPlugin
  {
    public:
      LeftEncoderPlugin() {}

      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        _sdf->GetElement("joint_name")->Set("left_wheel_joint");
        _sdf->GetElement("topic_name")->Set("left_wheel_joint_encoder");

        EncoderPlugin::Load(_model, _sdf);
      }
  };

  GZ_REGISTER_MODEL_PLUGIN(LeftEncoderPlugin)

  class RightEncoderPlugin : public EncoderPlugin
  {
    public:
      RightEncoderPlugin() {}

      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        _sdf->GetElement("joint_name")->Set("right_wheel_joint");
        _sdf->GetElement("topic_name")->Set("right_wheel_joint_encoder");

        EncoderPlugin::Load(_model, _sdf);
      }
  };

  GZ_REGISTER_MODEL_PLUGIN(RightEncoderPlugin)
}
