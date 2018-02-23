#ifndef OPERATIONAL_SPACE_CONTROL_H
#define OPERATIONAL_SPACE_CONTROL_H

#include <rclcpp/rclcpp.hpp>

#include <controllers/chainfksolveracc_recursive.hpp>
#include <kdl/jntarrayacc.hpp>
#include <kdl/frames.hpp>
#include <kdl/frameacc.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <msg/JointStates.h>
#inculde <msg/CartesianTrajectory.h>
#inculde <msg/TorqueCommands.h>

#include <eigen_conversions/eigen_kdl.h>
#include <tf_conversions/tf_kdl.h>

#include <Eigen/Core>
#include <memory>

class OperationalSpaceControl: public rclcpp::Node{

public:

   OperationalSpaceControl();

   void on_new_trajectory(control_msgs::JointStates joint_states_msg, control_msgs::CartesianTrajectory trajectory_msg);

   ~OperationalSpaceControl();

protected:
   
   rclcpp::Publisher<control_msgs::JointStates> torque_commands_pub;
   rclcpp::Publisher<geometry_msgs::PoseArray> trajectory_generator_pub;
   
   rclcpp::Subscription<control_msgs::CartesianTrajectory> trajectory_sub;
   rclcpp::Subscription<control_msgs::JointStates> joint_states_sub;
   
   geometry_msgs::Pose ee_start_pose_msg, desired_task_pos_msg;
   geometry_msgs::PoseArray ee_start_pose_to_port;
   geometry_msgs::Twist desired_task_vel_msg, desired_task_acc_msg;
   control_msgs::JointState joint_states;
   control_msgs::TorqueCommands torque_commands_msg;

   std::string robot_description, ee_frame_name, root_link, tip_link;
   KDL::JntArray joint_start_pose, joint_pos, gravity, coriolis;
   KDL::JntArrayVel joint_vel_array, last_joint_vel_array;
   KDL::JntArrayAcc joint_acc_array;
   KDL::FrameVel current_task_vel_solver;
   KDL::FrameAcc current_task_acc_solver;
   int ee_frame_index;
   std::vector<double> Kp, Ki, Kd;
   KDL::Vector grav_vec;
   KDL::Chain kdl_chain;
   std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver;
   std::unique_ptr<KDL::ChainDynParam> chain_dynamic_params;
   std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver;
   std::unique_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver;
   std::unique_ptr<KDL::ChainFkSolverAcc_recursive> fk_acc_solver;
   KDL::JntSpaceInertiaMatrix joint_space_mass_mat, joint_space_mass_mat_inv;
   KDL::Frame current_task_pos, desired_task_pos, ee_start_pose;
   KDL::Twist current_task_vel, current_task_acc, pose_error;
   KDL::Twist desired_task_vel, desired_task_acc, PID_commands;
   Eigen::VectorXd torque_commands;
   KDL::Jacobian jacobian;
   Eigen::MatrixXd jacobian_transpose, op_space_mass_mat;
   Eigen::Matrix<double,6,1> pid_commands;
};

#endif