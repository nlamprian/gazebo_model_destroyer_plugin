gazebo_model_destroyer_plugin
---

A gazebo world plugin that exposes a ROS service for destroying a model.

![demo](assets/example.gif)

Usage
---

Add the following configuration in the world

```xml
<plugin name="gazebo_model_destroyer_plugin" filename="libgazebo_model_destroyer_plugin.so">
  <destroyService>gazebo/destroy_model</destroyService>
  <linearVel>7</linearVel>
  <angularVel>3</angularVel>
</plugin>
```

Then, once gazebo is started and the model is spawned, call the destroy service with the name of the model

```bash
rosservice call /gazebo/destroy_model "model_name: my_stupid_robot"
```
