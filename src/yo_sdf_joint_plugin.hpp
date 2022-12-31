//
// Created by yossi on 22/12/2021.
//

#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>


namespace gazebo
{
class YoSdfJointPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  bool init = false;
  std::unique_ptr<ros::NodeHandle> nh_;
  gazebo::physics::ModelPtr model_;
  sdf::ElementPtr sdf_;
  std::string robot_namespace_;
  
  std::vector<std::string> joint_names_;
  
  ros::Publisher joint_state_pub_;
  ros::Subscriber params_sub;
  std::map<std::string, gazebo::physics::JointPtr> joints_;
  std::map<std::string, double> joint_force_set_points;
  std::map<std::string, ros::Publisher> joint_pos_pubs;
  std::map<std::string, ros::Publisher> joint_vel_pubs;
  std::map<std::string, ros::Subscriber> subscribers_;
  
  sensor_msgs::JointState joint_state_msg_;
  std::vector<event::ConnectionPtr> connections;
  
  double dt_ = 1e-3;
  
  void getNamespace(const physics::ModelPtr &_model);
  void createInterfacesForJoint(const std::string &joint_name);
  
  void onUpdate();
  void onReset();
  void publishPosVel(const std::string &name, double position, double velocity);
  static void randomizePosition(physics::JointPtr &joint_ptr) ;
  void setupCallbacks();
  void setupJoints();
  void fetchWorldDt();
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(YoSdfJointPlugin)
}
