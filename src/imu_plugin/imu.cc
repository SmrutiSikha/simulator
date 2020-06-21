#include "gazebo/common/Plugin.hh"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "ros/ros.h"
#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <iostream>
#include <ros/package.h>
#include <sdf/sdf.hh>
#include <synchronizer/Combined.h>

namespace gazebo {
class ImuPublisher : public ModelPlugin {
private:
  math::Vector3 acceleration;
  math::Pose orientation;
  event::ConnectionPtr update_event_;
  physics::ModelPtr model;
  ros::Publisher imu_publisher;
  ros::NodeHandle *nh;
  math::Pose pose;

public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    if (!ros::isInitialized()) {
      std::cout << "ros is not isInitialized";
      return;
    }
    this->model = _model;
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "controller");
    nh = new ros::NodeHandle;
    imu_publisher = nh->advertise<synchronizer::Combined>("/combined", 100);

    update_event_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ImuPublisher::OnUpdate, this));
  }

public:
  void OnUpdate() {

    synchronizer::Combined msg;
    acceleration = this->model->GetWorldLinearAccel();
    orientation = this->model->GetWorldPose();
    msg.angular = {orientation.rot.GetRoll() * 180 / 3.14,
                   orientation.rot.GetPitch() * 180 / 3.14,
                   orientation.rot.GetYaw() * 180 / 3.14};
    msg.linear = {acceleration.x, acceleration.y, acceleration.z};
    // pose = this->model->GetPose();
    msg.depth = orientation.pos.x;

    imu_publisher.publish(msg);
  }
};

GZ_REGISTER_MODEL_PLUGIN(ImuPublisher);
}
