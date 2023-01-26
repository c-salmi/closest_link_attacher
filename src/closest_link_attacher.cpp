#include <vector>

#include <closest_link_attacher/Attach.h>
#include <closest_link_attacher/Detach.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

namespace gazebo {

class ClosestLinkAttacherPlugin : public WorldPlugin {

public:
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM(
          "A ROS node for Gazebo has not been initialized, unable to load "
          "plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in "
             "the gazebo_ros package)");
      return;
    }

    // Store the pointer to the world
    this->_world = _world;
    this->_physics = this->_world->Physics();

    this->_attach_service = this->_nh.advertiseService(
        "/gazebo/closest_link_attacher/attach",
        &ClosestLinkAttacherPlugin::attach_callback, this);
    this->_detach_service = this->_nh.advertiseService(
        "/gazebo/closest_link_attacher/detach",
        &ClosestLinkAttacherPlugin::detach_callback, this);
  }

  bool attach_callback(closest_link_attacher::Attach::Request &req,
                       closest_link_attacher::Attach::Response &res) {
    res.ok = this->_attachClosestLink(req.model, req.link);
    return res.ok;
  }

  bool detach_callback(closest_link_attacher::Detach::Request &req,
                       closest_link_attacher::Detach::Response &res) {
    if (_joint) {
      _joint->Detach();
      _joint->Reset();
      res.ok = true;
    } else {
      res.ok = false;
    }
    return res.ok;
  }

private:
  bool _attachClosestLink(const std::string referenceModelName,
                          const std::string referenceLinkName) {

    double minDistance = std::numeric_limits<double>::max();
    physics::LinkPtr closestLink;
    physics::ModelPtr closestModel;

    // Get the reference link
    auto referenceModel = this->_world->ModelByName(referenceModelName);

    if (!referenceModel) {
      ROS_ERROR_STREAM("Model" << referenceModelName << " not found in world!");
      return false;
    }

    auto referenceLink = referenceModel->GetLink(referenceLinkName);

    if (!referenceLink) {
      ROS_ERROR_STREAM("Link" << referenceLinkName << " is not part of model "
                              << referenceModelName);
      return false;
    }

    // Get all the models in the world
    std::vector<physics::ModelPtr> models = this->_world->Models();

    // Loop through all the models
    for (auto model : models) {
      // skip reference model
      if (model == referenceModel) {
        continue;
      }

      this->_UpdateClosest(referenceLink, closestLink, closestModel,
                           minDistance, model);

      for (auto nestedModel : model->NestedModels())
        this->_UpdateClosest(referenceLink, closestLink, closestModel,
                             minDistance, nestedModel);
    }

    // attach the closest link if below minDistance
    if (closestLink && minDistance < 0.10) {
      ROS_INFO_STREAM("Attaching link '"
                      << closestLink->GetName() << "' of model '"
                      << closestModel->GetName() << "' to link '"
                      << referenceLinkName << "' of model '"
                      << referenceModelName << "'");
      ROS_INFO_STREAM("Distance between links: " << minDistance);
      return this->_attach(closestModel, closestLink, referenceModel,
                           referenceLink);
    } else {
      return false;
    }
  }

  bool _attach(physics::ModelPtr &model, physics::LinkPtr &link,
               physics::ModelPtr &referenceModel,
               physics::LinkPtr &referenceLink) {
    if (!_joint) {
      _joint = this->_physics->CreateJoint("fixed", referenceModel);
    }
    _joint->Reset();
    _joint->Attach(referenceLink, link);
    _joint->Load(referenceLink, link, ignition::math::Pose3d());
    _joint->SetModel(model);
    return true;
  }

  void _UpdateClosest(const physics::LinkPtr &referenceLink,
                      physics::LinkPtr &closestLink,
                      physics::ModelPtr &closestModel, double &minDistance,
                      physics::ModelPtr model) {
    // Get all the links in the model
    std::vector<physics::LinkPtr> links = model->GetLinks();

    // Loop through all the links
    for (auto link : links) {
      // Calculate the distance between the link and the reference link

      double distance =
          (link->WorldPose().Pos() - referenceLink->WorldPose().Pos()).Length();

      // If the distance is smaller than the current minimum distance, update
      // the closest link
      if (distance < minDistance) {
        minDistance = distance;
        closestLink = link;
        closestModel = model;
      }
    }
  }

  // Pointer to the world
private:
  physics::WorldPtr _world;
  physics::PhysicsEnginePtr _physics;

  physics::JointPtr _joint;

  ros::NodeHandle _nh;
  ros::ServiceServer _attach_service;
  ros::ServiceServer _detach_service;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ClosestLinkAttacherPlugin);
} // namespace gazebo
