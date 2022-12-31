//
// Created by daniel on 12/31/22.
//

#include "force_plugin.hpp"

void gazebo::ForcePlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    ROS_FATAL_ONCE("Force plugin has arrived to the party !");

    this->model_ = _model;
    thruster_link_ = model_->GetLink("thruster");
    if (thruster_link_)
    {
        ROS_INFO("Thruster model is %s", thruster_link_->GetModel()->GetSDF()->GetDescription().c_str());
    } else
    {
        ROS_WARN("Couldn't find link thruster !!");
    };

    connections.emplace_back(event::Events::ConnectWorldUpdateBegin(
            std::bind(&::gazebo::ForcePlugin::onUpdate, this)));


    nh_ = std::make_unique<ros::NodeHandle>("/rocket_sim");

    thruster_force_sub_ = nh_->subscribe<std_msgs::Float64>("thruster/force", 10, &ForcePlugin::ThrusterForceCallback,
                                                            this);
    z_thrust_.data = 0.0;
}

void gazebo::ForcePlugin::onUpdate()
{
//    thruster_link_->SetForce(ignition::math::Vector3d(0,0,100));
    thruster_link_->AddLinkForce(ignition::math::Vector3d(0, 0, z_thrust_.data));
}

void gazebo::ForcePlugin::onReset()
{
    thruster_link_->SetForce(ignition::math::Vector3d(0, 0, 0));
}

void gazebo::ForcePlugin::ThrusterForceCallback(const std_msgs::Float64ConstPtr &msg)
{
    z_thrust_.data = msg->data;
}
