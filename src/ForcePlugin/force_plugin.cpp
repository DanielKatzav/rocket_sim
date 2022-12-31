//
// Created by daniel on 12/31/22.
//

#include "force_plugin.hpp"

void gazebo::ForcePlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    ROS_FATAL_ONCE("Force plugin has arrived to the party !");
    ROS_FATAL_ONCE("Testing without relaunch");

    this->model_ = _model;
    thruster_link_ = model_->GetLink("thruster");
    if (thruster_link_)
    {
        ROS_INFO("Thruster model is %s", thruster_link_->GetModel()->GetSDF()->GetDescription().c_str());
    }
    else{
        ROS_WARN("Couldn't find link thruster !!");
    };

    connections.emplace_back(event::Events::ConnectWorldUpdateBegin(
            std::bind(&::gazebo::ForcePlugin::onUpdate, this)));
}

void gazebo::ForcePlugin::onUpdate()
{
//    thruster_link_->SetForce(ignition::math::Vector3d(0,0,100));
    thruster_link_->AddLinkForce(ignition::math::Vector3d(0,0,100));
}

void gazebo::ForcePlugin::onReset()
{
    thruster_link_->SetForce(ignition::math::Vector3d(0,0,0));
}
