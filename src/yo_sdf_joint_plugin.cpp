//
// Created by yossi on 22/12/2021.
//

#include "yo_sdf_joint_plugin.hpp"

#include <string>
#include <vector>
#include <sstream>

#include <dynamic_reconfigure/ConfigDescription.h>

using namespace gazebo;

double fRand(double fMin, double fMax)
{
  double f = (double) rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

void YoSdfJointPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  
  model_ = _model;
  sdf_ = _sdf;
  std::string sdf_name;
  if (sdf_->HasElement("name"))
  {
    sdf_name = sdf_->GetElement("name")->Get<std::string>();
  }
  printf(
    "=======================\nLoading YoSdfJointPlugin for model: [%s] with SDF: [%s ~ %s]\n=======================\n",
    model_->GetName().c_str(),
    sdf_->GetName().c_str(),
    sdf_name.c_str());
  if (not sdf_->HasElement("joints"))
  {
    return;
  }
  fetchWorldDt();
  
  getNamespace(_model);
  if (not ros::isInitialized())
  {
    std::cerr << "ROS is not initialized!\n";
    return;
  }
  //std::this_thread::sleep_for(std::chrono::milliseconds(10000));
  std::cout << "ROS is initialized!\n";
  
  try
  {
    nh_ = std::make_unique<ros::NodeHandle>("/" + robot_namespace_);
    printf("Starting YoSdfJointPlugin in namespace: [%s]\n", nh_->getNamespace().c_str());
    joint_state_pub_ = nh_->advertise<sensor_msgs::JointState>("joint_states", 10);
    joint_state_msg_.header.frame_id = robot_namespace_;
  }
  catch (std::exception &e)
  {
    std::cerr << "Exception occured while loading YoSdfJointPlugin:\n" << e.what() << std::endl;
  }
  try
  {
    setupCallbacks();
  }
  catch (std::exception &e)
  {
    std::cerr << "Exception occured while setupCallbacks:\n" << e.what() << std::endl;
  }
  try
  {
    setupJoints();
  }
  catch (std::exception &e)
  {
    std::cerr << "Exception occured while loading setupJoints():\n" << e.what() << std::endl;
  }
}

void YoSdfJointPlugin::fetchWorldDt()
{
  auto element = sdf_->GetParent();
  while (element)
  {
    //printf("\tWith parent: %s\n", element->GetName().c_str());
    
    if (element->GetName() == "world")
    {
      //std::cout << "==============================" << std::endl;
      //std::cout << "--> " << element->GetFirstElement()->GetName() << std::endl;
      auto physics = element->GetElement("physics");
      auto max_step_size = physics->GetElement("max_step_size");
      //std::cout << "Got: " << physics->GetName() << std::endl;
      //std::cout << "Got: " << max_step_size->GetName() << std::endl;
      //std::cout << " --> max_step_size = " << max_step_size->ToString(">") << std::endl;
      //std::cout << " --> max_step_size = " << max_step_size->Get<double>() << std::endl;
      //std::cout << "==============================" << std::endl;
      
      dt_ = max_step_size->Get<double>();
      break;
    }
    
    element = element->GetParent();
    
  }
}

void YoSdfJointPlugin::getNamespace(const physics::ModelPtr &_model)
{// Get namespace for nodehandle
  if (sdf_->HasElement("robotNamespace"))
  {
    robot_namespace_ = sdf_->GetElement("robotNamespace")->Get<std::string>();
    std::cout << "Using SDF argument for robotNamespace: " << robot_namespace_ << std::endl;
  }
  else
  {
    robot_namespace_ = _model->GetName(); // default
    std::cout << "Using SDF model name for robotNamespace: " << robot_namespace_ << std::endl;
  }
}

void YoSdfJointPlugin::setupCallbacks()
{
  connections.emplace_back(event::Events::ConnectWorldUpdateBegin(
    std::bind(&YoSdfJointPlugin::onUpdate, this)));
  connections.emplace_back(event::Events::ConnectWorldReset(
    std::bind(&YoSdfJointPlugin::onReset, this)));
}

void YoSdfJointPlugin::setupJoints()
{
  auto joint_names = sdf_->GetElement("joints")->Get<std::string>();
  std::stringstream ss_joint_names(joint_names);
  printf("===>> Got 'joints' parameter: '%s'\n", joint_names.c_str());
  std::string joint_name;
  bool random_position = sdf_->HasElement("random_position") and sdf_->GetElement("random_position")->Get<bool>();
  while (std::getline(ss_joint_names, joint_name, ' '))
  {
    
    auto joint_ptr = model_->GetJoint(joint_name);
    if (not joint_ptr)
    {
      std::cerr << "Requested joint not found: " << joint_name << std::endl;
      continue;
    }
    printf("\t\tAdding joint: [%s]\n", joint_name.c_str());
    if (random_position)
    {
      randomizePosition(joint_ptr);
    }
    //printf("Creating interface for joint %s\n", joint_name.c_str());
    joint_names_.push_back(joint_name);
    joints_[joint_name] = joint_ptr;
    createInterfacesForJoint(joint_name);
  }
}

void YoSdfJointPlugin::createInterfacesForJoint(const std::string &joint_name)
{
  joint_pos_pubs[joint_name] = nh_->advertise<std_msgs::Float64>(joint_name + "/pos", 10);
  joint_vel_pubs[joint_name] = nh_->advertise<std_msgs::Float64>(joint_name + "/vel", 10);
  
  boost::function<void(const std_msgs::Float64 &)> callback = [=](const std_msgs::Float64 &msg)
  {
    joint_force_set_points[joint_name] = msg.data;
    //joints_[joint_name]->SetForce(0, msg.data);
    //printf("Received command for joint %s: %+9.4f\n", joint_name.c_str(), msg.data);
  };
  subscribers_[joint_name] = nh_->subscribe<std_msgs::Float64>(
    joint_name + "/force_cmd", 10, callback
  );
}

void YoSdfJointPlugin::randomizePosition(physics::JointPtr &joint_ptr)
{
  auto lower = joint_ptr->LowerLimit(0);
  auto upper = joint_ptr->UpperLimit(0);
  auto new_position = fRand(lower, upper);
  printf("Randomizing a position for joint %s -> %6.3f\n", joint_ptr->GetName().c_str(), new_position);
  joint_ptr->SetPosition(0, new_position);
}

void YoSdfJointPlugin::onReset()
{
  for (auto&[name, joint]: joints_)
  {
    randomizePosition(joint);
  }
  
}

void YoSdfJointPlugin::onUpdate()
{
  joint_state_msg_.header.stamp = ros::Time::now();
  joint_state_msg_.name.clear();
  joint_state_msg_.position.clear();
  joint_state_msg_.velocity.clear();
  joint_state_msg_.effort.clear();
  for (const auto&[name, joint]: joints_)
  {
    double effort = joint_force_set_points[name];// * dt_;
    joint->SetForce(0, effort);
    auto position = joint->Position(0);
    auto velocity = joint->GetVelocity(0);
    
    joint_state_msg_.name.emplace_back(name);
    joint_state_msg_.position.emplace_back(position);
    joint_state_msg_.velocity.emplace_back(velocity);
    joint_state_msg_.effort.emplace_back(effort);
    
    publishPosVel(name, position, velocity);
  }
  joint_state_pub_.publish(joint_state_msg_);
  
}

void YoSdfJointPlugin::publishPosVel(const std::string &name, double position, double velocity)
{
  std_msgs::Float64 msg;
  msg.data = position;
  joint_pos_pubs[name].publish(msg);
  msg.data = velocity;
  joint_vel_pubs[name].publish(msg);
}
