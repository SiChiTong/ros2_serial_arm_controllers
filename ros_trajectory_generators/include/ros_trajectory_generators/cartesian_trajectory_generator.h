#ifndef CARTESIAN_TRAJECTROY_GENERATOR_H
#define CARTESIAN_TRAJECTROY_GENERATOR_H


#include <ros_trajectory_generators/cart_trajectory.h>

#include <rclcpp/rclcpp.hpp>
#include <class_loader/register_macro.hpp>

#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/path_point.hpp>
#include <kdl/framevel.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/utilities/error.h>

#include <tf_conversions/tf_kdl.h>
#include <tf/transform_listener.h>
#include <tf/exceptions.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <msg/CartesianTrajectory>

#include <memory>

class CartesianTrajectory: public rclcpp::Node{

public:
  
    CartesianTrajectory();
    
    bool computeTrajectory();
    void update(const geometry_msgs::PoseArray& waypoint_array);
    
    ~CartesianTrajectory();

protected:
   
   rclcpp::Publisher<control_msgs::CartesianTrajectory> cart_trajectory;
   
   rclcpp::Subscription<geometry_msgs::PoseStamped> trajectory_sub;
   
   geometry_msgs::PoseArray waypoint_array, transformed_waypoint_array;
   geometry_msgs::PoseStamped pose_stamped;
   control_msgs::CartesianTrajectory cart_trajectory_msg;
   
   bool trajectory_computed, transform_points;
   std::string frame_id, task_space_velocity_profile;
   double start_time, current_trajec_time, task_space_vel_limit, task_space_acc_limit;
   double task_trajectory_corner_radius, task_trajectory_equivalent_radius;
   KDL::Path_RoundedComposite* path;
   KDL::Trajectory* traject;
   KDL::Trajectory_Composite* comp_trajec;
   KDL::VelocityProfile* vel_profile;
   KDL::RotationalInterpolation_SingleAxis* interpolator;
   KDL::Frame frame, previous_frame, current_pos;
   KDL::Twist current_vel, current_acc, frame_diff;
   std::vector<KDL::Frame> frame_waypoint_vector;
   tf::TransformListener tf_listener;
};

#endif
