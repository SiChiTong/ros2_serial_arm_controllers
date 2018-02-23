#ifndef RANDOM_WAYPOINT_ARRAY_GENERATOR_H
#define RANDOM_WAYPOINT_ARRAY_GENERATOR_H

#include <rclcpp/rclcpp.hpp>

#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <tf_conversions/tf_kdl.h>

#include <memory>
#include <random>

class RandomWaypointArrayGenerator: public rclcpp::Node{

public:
   
   RandomWaypointArrayGenerator();
   
   generate();
   
   ~RandomWaypointArrayGenerator(){}

protected:
   
   rclcpp::Publisher<geometry_msgs::PoseArray> waypoint_vector_pub;
   geometry_msgs::PoseArray waypoint_vector;
   
   std::string robot_description, root_link, tip_link, frame_id;
   KDL::Frame goal_pose_frame;
   KDL::Chain kdl_chain;
   std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver;
   KDL::JntArray rand_joint_angles;
   std::vector<double> joint_lower_lims;
   std::vector<double> joint_upper_lims;
   int num_of_points;
   double lower_lim, upper_lim;
   std::random_device random;
   std::uniform_real_distribution<double> angles; 
};

#endif
