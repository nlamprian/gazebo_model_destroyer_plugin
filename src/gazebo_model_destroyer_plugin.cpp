#include <gazebo_model_destroyer_plugin/gazebo_model_destroyer_plugin.h>

namespace gazebo {

GazeboModelDestroyerPlugin::GazeboModelDestroyerPlugin()
    : spinner_(1, &callback_queue_), gen_(rd_()) {}

GazeboModelDestroyerPlugin::~GazeboModelDestroyerPlugin() {}

void GazeboModelDestroyerPlugin::Load(physics::WorldPtr _world,
                                      sdf::ElementPtr _sdf) {
  this->world_ = _world;

  // Initialize linear velocity distribution
  if (not _sdf->HasElement("linearVel")) {
    ROS_ERROR_STREAM_NAMED(
        "model_destroyer",
        "GazeboModelDestroyerPlugin: Failed to get linearVel");
    return;
  }
  double linear_vel = _sdf->GetElement("linearVel")->Get<double>();
  uni_dist_lin_ = std::uniform_real_distribution<>(-linear_vel, linear_vel);

  // Initialize angular velocity distribution
  if (not _sdf->HasElement("angularVel")) {
    ROS_ERROR_STREAM_NAMED(
        "model_destroyer",
        "GazeboModelDestroyerPlugin: Failed to get angularVel");
    return;
  }
  double angular_vel = _sdf->GetElement("angularVel")->Get<double>();
  uni_dist_ang_ = std::uniform_real_distribution<>(-angular_vel, angular_vel);

  if (not ros::isInitialized()) {
    ROS_FATAL_STREAM(
        "A ROS node for Gazebo has not been initialized, unable to load "
        "plugin. Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' "
        "in the gazebo_ros package)");
    return;
  }

  nh_ = boost::make_shared<ros::NodeHandle>();
  nh_->setCallbackQueue(&callback_queue_);
  spinner_.start();

  // Initialize destroy service
  if (not _sdf->HasElement("destroyService")) {
    ROS_ERROR_NAMED("model_destroyer",
                    "GazeboModelDestroyerPlugin: Failed to get destroyService");
    return;
  }
  std::string destroy_service_name = _sdf->Get<std::string>("destroyService");
  destroy_service_ = nh_->advertiseService(
      destroy_service_name, &GazeboModelDestroyerPlugin::destroyServiceCallback,
      this);
}

ignition::math::Vector3d GazeboModelDestroyerPlugin::getRandomVel(
    bool is_linear) {
  ignition::math::Vector3d vel;
  if (is_linear) {
    vel.X() = uni_dist_lin_(gen_);
    vel.Y() = uni_dist_lin_(gen_);
    vel.Z() = uni_dist_lin_(gen_);
  } else {
    vel.X() = uni_dist_ang_(gen_);
    vel.Y() = uni_dist_ang_(gen_);
    vel.Z() = uni_dist_ang_(gen_);
  }
  return vel;
}

bool GazeboModelDestroyerPlugin::destroyServiceCallback(
    gazebo_model_destroyer_plugin::DestroyRequest& req,
    gazebo_model_destroyer_plugin::DestroyResponse& res) {
  auto model = world_->ModelByName(req.model_name);
  if (model == nullptr) {
    ROS_WARN_STREAM_NAMED(
        "model_destroyer",
        "GazeboModelDestroyerPlugin: Failed to get model " << req.model_name);
    res.success = false;
    return true;
  }

  for (auto& joint : model->GetJoints()) {
    joint->Detach();
  }
  for (auto& link : model->GetLinks()) {
    link->SetLinearVel(getRandomVel());
    link->SetAngularVel(getRandomVel(false));
  }

  res.success = true;
  return true;
}

GZ_REGISTER_WORLD_PLUGIN(GazeboModelDestroyerPlugin)

}  // namespace gazebo
