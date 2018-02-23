#include <ros_trajectory_generators/random_waypoint_array_generator.h>

RandomWaypointArrayGenerator::RandomWaypointArrayGenerator(): rclcpp::Node("random_waypoint_array")
{
   waypoint_vector_pub = this->create_publisher<geometry_msgs::PoseArray>("trajectory", 1000);
   
   // Create forward kinematics solver.
   fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));
   
   // Resize.
   joint_lower_lims.resize(kdl_chain.getNrOfJoints());
   joint_lower_lims.resize(kdl_chain.getNrOfJoints());
   rand_joint_angles.resize(kdl_chain.getNrOfJoints());
}

// Run update loop.
void RandomWaypointArrayGenerator::update()
{
    // Generate a random number of waypoints. Arbitrary limit set to 10.
    std::uniform_int_distribution<int> points(1, 10);
    
    num_of_points = points(random);

    waypoint_vector.poses.resize(num_of_points);

    // For each trajectory waypoint:
    for(int i = 0; i<num_of_points; ++i)
    {
        // For each joint, generate a random angle within limits.
        for(int i = 0; i<joint_lower_lims.size(); ++i)
        {
            std::uniform_real_distribution<double> angles(joint_lower_lims[i], joint_upper_lims[i]);
            rand_joint_angles(i) = angles(random);
        }
        // solve forward kinematics to find the cartesian pose.
        fk_pos_solver->JntToCart(rand_joint_angles, goal_pose_frame);

	// Convert KDL frame to pose message.
	tf::poseKDLToMsg(goal_pose_frame, waypoint_vector.poses[i]);
    }
    // Set frame id.
    waypoint_vector.header.frame_id = frame_id;
    
    // Set time stamp.
    waypoint_vector.header.stamp = rclcpp::Clock::now();
    
    // Publish waypoint vector.
    waypoint_vector_pub.publish(waypoint_vector);
}

CLASS_LOADER_REGISTER_CLASS(RandomWaypointArrayGenerator, rclcpp::Node);