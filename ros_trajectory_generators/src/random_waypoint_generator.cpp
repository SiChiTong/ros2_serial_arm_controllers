#include <ros_trajectory_generators/random_waypoint_generator.h>

RandomWaypointGenerator::RandomWaypointGenerator(): rclcpp::Node("random_waypoints") 
{   
   // Resize.
   rand_joint_angles.resize(kdl_chain.getNrOfJoints());
   joint_lower_lims.resize(kdl_chain.getNrOfJoints());
   joint_upper_lims.resize(kdl_chain.getNrOfJoints());
   
   // Create fk solver.
   fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));
   
   waypoint_pub = this->create_publisher<geometry_msgs::PoseStamped>("trajectory", 1000);
}

// Run update loop.
void RandomWaypointGenerator::update()
{
    // Generate random joint positions within joint limits.
    for(int i = 0; i<joint_lower_lims.size(); ++i)
    {
      std::uniform_real_distribution<double> angles(joint_lower_lims[i], joint_upper_lims[i]);
      rand_joint_angles(i) = angles(random);
    }
    // Solve forward kinematics to find the cartesian pose.
    fk_pos_solver->JntToCart(rand_joint_angles, trajectory_goal_frame);
    
    // Convert KDL frame to pose message.
    tf::poseKDLToMsg(trajectory_goal_frame, trajectory_goal_pose.pose);
    
    // Set frame id.
    trajectory_goal_pose.header.frame_id = frame_id;
    
    // Set time stamp.
    trajectory_goal_pose.header.stamp = ros::Time::now();
    
    //send cartesian pose.
    waypoint_pub.publish(trajectory_goal_pose);
}

CLASS_LOADER_REGISTER_CLASS(RandomWaypointGenerator, rclcpp::Node);