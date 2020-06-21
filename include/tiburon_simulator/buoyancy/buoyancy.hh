#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Shape.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

namespace gazebo {
class BuoyancyPlugin : public ModelPlugin {
public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate();

private:
  struct link_st {
    std::string model_name;
    physics::LinkPtr link;
    math::Vector3 buoyant_force;
    math::Vector3 buoyancy_center;
    math::Vector3 linear_damping;
    math::Vector3 angular_damping;
    double limit;
  };
  struct model_st {
    std::string name;
    physics::ModelPtr model_ptr;
  };

  // parse a Vector3 string
  void ReadVector3(const std::string &_string, math::Vector3 &_vector);
  // parse a new model
  void ParseNewModel(const physics::ModelPtr &_model);
  // removes a deleted model
  void RemoveDeletedModel(std::vector<model_st>::iterator &_model_it);

private:
  // plugin options
  bool has_surface_;
  math::Vector4 surface_plane_;
  std::string description_;

  physics::WorldPtr world_;
  event::ConnectionPtr update_event_;

  // links that are subject to fluid effects
  std::vector<link_st> buoyant_links_;
  // models that have been parsed
  std::vector<model_st> parsed_models_;

  math::Vector3 fluid_velocity_;
};

GZ_REGISTER_MODEL_PLUGIN(BuoyancyPlugin);
}
