#ifndef GAZEBO_MODEL_DESTROYER_PLUGIN_H
#define GAZEBO_MODEL_DESTROYER_PLUGIN_H

#include <random>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo_model_destroyer_plugin/Destroy.h>

namespace gazebo {

/**
 * @brief Simulates an explosion of a model.
 * @details Exposes a ROS service that given a model name, it destroys the
 * joints of the model and gives random twist velocities to the links, so it
 * looks like there are an explosion.
 * @note Expects the following configuration:
 * * destroyService: name of the service
 * * linearVel: maximum linear velocity for the links
 * * angularVel: maximum angular velocity for the links
 */
class GazeboModelDestroyerPlugin : public WorldPlugin {
 public:
  GazeboModelDestroyerPlugin();
  virtual ~GazeboModelDestroyerPlugin();
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

 private:
  ignition::math::Vector3d getRandomVel(bool is_linear = true);
  bool destroyServiceCallback(
      gazebo_model_destroyer_plugin::DestroyRequest &req,
      gazebo_model_destroyer_plugin::DestroyResponse &res);

  physics::WorldPtr world_;

  ros::NodeHandlePtr nh_;
  ros::CallbackQueue callback_queue_;
  ros::AsyncSpinner spinner_;

  ros::ServiceServer destroy_service_;

  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<> uni_dist_lin_;
  std::uniform_real_distribution<> uni_dist_ang_;
};

}  // namespace gazebo

#endif  // GAZEBO_MODEL_DESTROYER_PLUGIN_H
