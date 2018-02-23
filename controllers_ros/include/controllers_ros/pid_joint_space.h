#ifndef PID_JOINT_SPACE_H
#define PID_JOINT_SPACE_H

#include <rclcpp/rclcpp.hpp>

#include <ros_trajectory_generators/joint_space_velocity_profiles.h>

#include <kdl/jntarrayacc.hpp>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <msg/JointStates.h>
#inculde <msg/TorqueCommands.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>

#include <Eigen/Core>
#include <memory>
#include <functional>

class PIDjointSpace: public rclcpp::Node{

public:

   PIDjointSpace();
   
   void on_new_trajectory(control_msgs::JointStates joint_states_msg, geometry_msgs::PoseStamped trajectory_goal_pose);
   
   ~PIDjointSpace();

protected:
   
   rclcpp::Publisher<control_msgs::TorqueCommands> torque_commands_pub;
   
   rclcpp::Subscription<geometry_msgs::PoseStamped> trajectory_sub;
   rclcpp::Subscription<control_msgs::JointStates> joint_states_sub;
   
   control_msgs::TorqueCommands torque_commands;
   control_msgs::JointStates joint_states_msg;
   geometry_msgs::PoseStamped trajectory_goal_pose;

   bool use_joint_vel_lims, use_joint_acc_lims, transform_points;
   double current_time, duration, max_solve_time, old_timestamp, start_time, error;
   std::string robot_description, root_link, tip_link, joint_space_velocity_profile, frame_id;
   std::vector<double> Kp, Ki, Kd, joint_vel_lims, joint_acc_lims;
   KDL::Chain kdl_chain;
   KDL::Frame trajectory_goal_frame;
   KDL::JntArray joint_start_pose, joint_final_pose;
   KDL::JntArrayVel joint_states, desired_joint_states;
   
   std::unique_ptr<joint_space_vel_prof::velocityProfile> vel_prof;
   std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver;
   tf::TransformListener tf_listener;
};

#endif
