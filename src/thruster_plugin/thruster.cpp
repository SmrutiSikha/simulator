#include "gazebo/common/Plugin.hh"
#include "ros/ros.h"
#include "thruster_controller/ThrusterSpeeds.h"
#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <iostream>
#include <ros/package.h>
#include <sdf/sdf.hh>

using namespace std;
namespace gazebo {
class Thruster : public ModelPlugin {
private:
  ros::NodeHandle *nh;
  ros::Subscriber thrusterSpeedSub;
  double force[6];
  physics::ModelPtr model;
  physics::LinkPtr thruster[6];
  double adjusted = 0.0f;

public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    if (!ros::isInitialized()) {
      std::cout << "ros is not isInitialized";
      return;
    }
    this->model = _model;
    for (int i = 0; i < 6; i++) {
      this->thruster[i] = _model->GetChildLink("tiburon_bot_final::thruster_" +
                                               to_string(i + 1));
    }
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "thruster_controller");
    nh = new ros::NodeHandle;
    thrusterSpeedSub = nh->subscribe("/thruster_speeds", 10,
                                     &Thruster::thruster_callback, this);
    // for (int i = 0; i < 6; i++) {
    //   k = std::to_string(i + 1);
    //   this->thruster[i] = _model->GetChildLink(thruster_ + k);
    // }
  }

public:
  double forceCalculation(int initial) {
    if (initial >= 1470 && initial <= 1530) {
      adjusted = 0.0f;
    } else if (initial > 1530) {
      initial = initial - 1530;
      adjusted = (initial / 370.0f) * 2.36f;
    } else {
      initial -= 1470;
      adjusted = initial / 370.0f * 1.85f;
    }
    adjusted *= 9.8f * 1000.0f;
    return adjusted;
  }

public:
  void gazeboThrustersForce() {
    this->thruster[0]->AddRelativeForce(math::Vector3(0, 0, -force[0]));
    this->thruster[1]->AddRelativeForce(math::Vector3(0, 0, -force[1]));
    this->thruster[2]->AddRelativeForce(math::Vector3(0, 0, -force[2]));
    this->thruster[3]->AddRelativeForce(math::Vector3(force[3], 0, 0));
    this->thruster[4]->AddRelativeForce(math::Vector3(force[4], 0, 0));
    this->thruster[5]->AddRelativeForce(math::Vector3(0, -force[5], 0));
  }

public:
  void
  thruster_callback(const thruster_controller::ThrusterSpeeds::ConstPtr &msg) {
    for (int i = 0; i < 6; i++) {
      force[i] = forceCalculation(msg->data[i]);
    }
    gazeboThrustersForce();
  }
};
GZ_REGISTER_MODEL_PLUGIN(Thruster);
}
