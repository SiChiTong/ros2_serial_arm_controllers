#ifndef RANDOM_WAYPOINT_GENERATOR_H
#define RANDOM_WAYPOINT_GENERATOR_H

#include <rclcpp/rclcpp.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_kdl.h>

#include <random>
#include <memory>

class RandomWaypointGenerator: public rclcpp::Node
{

public:
   
   RandomWaypointGenerator();
   
   void generate();
   
   ~RandomWaypointGenerator(){}

protected:
   
   rclcpp::Publisher<geometry_msgs::PoseStamped> waypoint_pub;
   
   geometry_msgs::PoseStamped trajectory_goal_pose;
   
   std::string robot_description, root_link, tip_link, frame_id;
   KDL::Frame trajectory_goal_frame;
   KDL::Chain kdl_chain;
   std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver;
   std::uniform_real_distribution<double> angles;
   KDL::JntArray rand_joint_angles;
   std::vector<double> joint_lower_lims;
   std::vector<double> joint_upper_lims;
   std::random_device random;   
};

#endif 