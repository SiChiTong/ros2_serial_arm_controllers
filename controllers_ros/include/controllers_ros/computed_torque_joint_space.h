#ifndef COMPUTED_TORQUE_JOINT_SPACE_H
#define COMPUTED_TORQUE_JOINT_SPACE_H

#include <rclcpp/rclcpp.hpp>

#include <trajectory_generators/joint_space_velocity_profiles.h>

#include <kdl/jntarray.hpp>
#include <kdl/jntarrayacc.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp>

#include <eigen_conversions/eigen_kdl.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <msg/JointStates.h>
#inculde <msg/TorqueCommands.h>

#include <Eigen/Core>
#include <memory>

class ComputedTorqueJointSpace: public rclcpp::Node{

public:

   ComputedTorqueJointSpace();
   
   void on_new_trajectory(control_msgs::JointStates joint_states_msg, geometry_msgs::PoseStamped trajectory_goal_pose);

   ~ComputedTorqueJointSpace();

protected:
   
   rclcpp::Publisher<control_msgs::TorqueCommands> torque_commands_pub;
   
   rclcpp::Subscription<geometry_msgs::PoseStamped> trajectory_sub;
   rclcpp::Subscription<control_msgs::JointStates> joint_states_sub;

   control_msgs::JointStates joint_states_msg;
   control_msgs::TorqueCommands torque_commands_msg;
   geometry_msgs::PoseStamped trajectory_goal_pose;
   
   bool use_joint_vel_lims, use_joint_acc_lims, transform_points;
   double start_time, current_time, old_timestamp, duration, max_solve_time, error;
   std::vector<double> Kp, Kd, joint_vel_lims, joint_acc_lims;
   KDL::Vector grav_vec;
   KDL::JntArrayVel joint_states;
   KDL::JntArrayAcc desired_joint_states;
   KDL::JntArray joint_start_pos, joint_final_pos;
   KDL::JntArray coriolis, gravity, dynam_commands, PID_commands;
   KDL::Chain kdl_chain;
   std::unique_ptr<KDL::ChainDynParam> chain_dynamic_params;
   KDL::JntSpaceInertiaMatrix mass_mat;
   KDL::Frame trajectory_goal_frame;
   std::unique_ptr<joint_space_vel_prof::velocityProfile> vel_prof;
   std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver;
   std::string robot_description, root_link, tip_link, frame_id;
   std::string vel_prof_name, joint_space_velocity_profile;
   Eigen::VectorXd torque_commands;
   tf::TransformListener tf_listener;
};

#endif

