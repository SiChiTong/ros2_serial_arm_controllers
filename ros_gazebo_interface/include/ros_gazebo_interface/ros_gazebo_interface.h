#ifndef ROS_GAZEBO_INTERFACE_H
#define ROS_GAZEBO_INTERFACE_H

#include <rclcpp/rclcpp.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include <kdl/chain.hpp>

#include <msg/JointState.h>
#include <msg/TorqueCommands.h>

#include <functional>
#include <thread>
#include <fstream>
#include <assert.h>


class GazeboInterface: public rclcpp::Node{
   
public:
   
   GazeboInterface();
   
   void addPlugin(const std::string& filename);
   bool startGazebo();
   bool spawnModel(const std::string& model_description);
   void resetWorld();
   void WorldUpdateBegin();
   void WorldUpdateEnd();
   void on_new_command(control_msgs::JointState torque_commands);
   
   ~GazeboInterface();
   
protected:
   
   rclcpp::Publisher<sensor_msgs::JointState> joint_states_pub;
   
   rclcpp::Subscription<sensor_msgs::JointState> torque_commands_sub;
   
   control_msgs::JointState joint_states;
   control_msgs::TorqueCommands torque_commands;
   
   gazebo::physics::ModelPtr model;
   gazebo::physics::WorldPtr loaded_world;
   std::vector<std::string> argv;
   std::vector<std::string> gazebo_joint_names;
   std::vector<gazebo::physics::JointPtr> gazebo_joints;
   std::string robot_description, first_joint, last_joint;
   std::string model_name, gazebo_world;
   int first_joint_index, last_joint_index;
   std::thread gz_thread;
   KDL::Chain kdl_chain;
};

#endif
