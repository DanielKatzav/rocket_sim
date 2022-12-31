//
// Created by daniel on 12/31/22.
//

#ifndef ROCKET_SIM_FORCE_PLUGIN_HPP
#define ROCKET_SIM_FORCE_PLUGIN_HPP

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>


namespace gazebo
{
    class ForcePlugin : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    private:
        bool init = false;
        std_msgs::Float64 z_thrust_;
        std::unique_ptr<ros::NodeHandle> nh_;
        gazebo::physics::ModelPtr model_;
        gazebo::physics::LinkPtr thruster_link_;
        sdf::ElementPtr sdf_;
        std::string robot_namespace_;

        ros::Subscriber thruster_force_sub_;
        std::vector<event::ConnectionPtr> connections;

        void getNamespace(const physics::ModelPtr &_model);

        void onUpdate();
        void onReset();

        void ThrusterForceCallback(const std_msgs::Float64ConstPtr& msg);
    };

// Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ForcePlugin)
}


#endif //ROCKET_SIM_FORCE_PLUGIN_HPP
