#ifndef PID_TASK_SPACE_H
#define PID_TASK_SPACE_H

#include <rclcpp/rclcpp.hpp>

#include </home/dan/ros2_my_code/src/ros_serial_arm_controllers/ros_tools/include/ros_tools/ros_tools.h>
#include <ros/ros.h>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <eigen_conversions/eigen_kdl.h>
#include <tf_conversions/tf_kdl.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <msg/JointStates.h>
#inculde <msg/CartesianTrajectory.h>
#inculde <msg/TorqueCommands.h>

#include <Eigen/Core>
#include <memory>

class PIDtaskSpace: public rclcpp::Node{

public:

   PIDtaskSpace();
   
   void on_new_trajectory(control_msgs::JointStates joint_states_msg, control_msgs::CartesianTrajectory trajectory_msg);
   
   ~PIDtaskSpace();

protected:
   
   rclcpp::Publisher<control_msgs::TorqueCommands> torque_commands_pub;
   rclcpp::Publisher<geometry_msgs::PoseArray> trajectory_generator_pub;
   
   rclcpp::Subscription<control_msgs::CartesianTrajectory> trajectory_sub;
   rclcpp::Subscription<control_msgs::JointStates> joint_states_sub;
   
   control_msgs::JointStates joint_states_msg;
   control_msgs::TorqueCommands torque_commands_msg;
   geometry_msgs::Pose ee_start_pose_msg;
   geometry_msgs::PoseArray ee_start_pose_to_port;
   geometry_msgs::Pose current_joint_pos_msg, desired_task_pos_msg;
   geometry_msgs::Twist current_joint_vel_msg, desired_task_vel_msg;
   control_msgs::CartesianTrajectory trajectory_msg;

   KDL::Chain kdl_chain;
   std::string robot_description, root_link, tip_link, ee_frame_name;
   int ee_frame_index;
   std::vector<double> Kp, Ki, Kd;
   KDL::JntArray joint_start_pos;
   KDL::JntArrayVel joint_states;
   KDL::Frame current_task_pos, desired_task_pos, ee_start_pose_frame;
   KDL::Twist current_task_vel, desired_task_vel, pose_error, PID_commands;
   KDL::FrameVel current_task_vel_solver;
   std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver;
   std::unique_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver;
   std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver;
   KDL::Jacobian jacobian;
   Eigen::MatrixXd jacobian_transpose;
   Eigen::VectorXd torque_commands;
   Eigen::Matrix<double,6,1> pid_commands;
};

#endif