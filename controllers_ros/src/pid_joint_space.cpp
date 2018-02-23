#include </home/dan/ros2_my_code/src/ros_serial_arm_controllers/controllers_ros/include/controllers_ros/pid_joint_space.h>

PIDjointSpace::PIDjointSpace(): rclcpp::Node("pid_joint_space")
{
   // Get KDL chain
   ros_tools::getChainFromURDF(kdl_chain);
   
   // Resize.
   Kp.resize(kdl_chain.getNrOfJoints());
   Ki.resize(kdl_chain.getNrOfJoints());
   Kd.resize(kdl_chain.getNrOfJoints());
   joint_vel_lims.resize(kdl_chain.getNrOfJoints());
   joint_acc_lims.resize(kdl_chain.getNrOfJoints());
   joint_start_pose.resize(kdl_chain.getNrOfJoints());
   joint_final_pose.resize(kdl_chain.getNrOfJoints());
   joint_states.q.resize(kdl_chain.getNrOfJoints());
   joint_states.qdot.resize(kdl_chain.getNrOfJoints());
   desired_joint_states.q.resize(kdl_chain.getNrOfJoints());
   desired_joint_states.qdot.resize(kdl_chain.getNrOfJoints());
   joint_states_msg.position.resize(kdl_chain.getNrOfJoints()); 
   joint_states_msg.velocity.resize(kdl_chain.getNrOfJoints());
   torque_commands.effort.resize(kdl_chain.getNrOfJoints());
   
   // Parse parameters.
   ros_tools::getJointSpaceVelProfileName(joint_space_velocity_profile);
   joint_space_vel_prof::getJointSpaceVelProfile(joint_space_velocity_profile, kdl_chain, vel_prof);
   ros_tools::getJointSpacePgains(kdl_chain, Kp);
   ros_tools::getJointSpaceIgains(kdl_chain, Ki);
   ros_tools::getJointSpaceDgains(kdl_chain, Kd);
   ros_tools::getIKsolver(ik_solver;
   ros_tools::getJointStartPose(kdl_chain, joint_start_pose);
   ros_tools::transformWaypointsToNewFrame(transform_points);
   if(transform_points) ros_tools::getFrameID(frame_id);

   torque_commands_pub = this->create_publisher<sensor_msgs::JointState>("torque_commands", 1000);
   
   trajectory_sub = this->create_subscription<geometry_msgs::PoseStamped>("trajectory", std::bind(on_new_trajectory, trajectory_goal_pose, _1);
   joint_states_sub = this->create_subscription<sensor_msgs::JointState>("joint_states", std::bind(on_new_trajectory, _1, joint_states_msg);
}

// Run update loop.
void PIDjointSpace::on_new_trajectory(sensor_msgs::JointState joint_states_msg, geometry_msgs::PoseStamped trajectory_goal_pose)
{ 
   // Convert joint state message to KDL JntArrayVel.
   joint_states.q.data = Eigen::VectorXd::Map(joint_states_msg.position.data(), joint_states_msg.position.size());
   joint_states.qdot.data = Eigen::VectorXd::Map(joint_states_msg.velocity.data(), joint_states_msg.velocity.size());
   
   if(trajectory_goal_pose.header.stamp != old_timestamp){
      
      if(transform_points)
	 tf_listener.transformPose(frame_id, trajectory_goal_pose, trajectory_goal_pose);
      
      // Convert pose message to KDL frame.
      tf::poseMsgToKDL(trajectory_goal_pose.pose, trajectory_goal_frame);
      
      // Find an IK solution.
      ik_solver->CartToJnt(joint_states.q, trajectory_goal_frame, joint_final_pose);
      
      old_timestamp = trajectory_goal_pose.header.stamp;
   }
   // Set to start position if there are no new goal poses.
   else joint_final_pose.data = joint_start_pose.data;
									 
   // Set up velocity profile.
   vel_prof->solve(joint_states.q, joint_final_pose, duration);
   
   // Reset reference time before beginning new trajectory.
   start_time = rclcpp::Clock::now();
         
   while(current_time <= duration){
      
      // update timer.
      current_time = rclcpp::Clock::now() - start_time;
      
      // Get desired joint positions.
      vel_prof->get_desired_joint_pos(desired_joint_states.q, current_time);
      
      // Get desired joint velocities.
      vel_prof->get_desired_joint_vel(desired_joint_states.qdot, current_time);
      
      // Create torque commands message.
      for(int i=0; i<joint_states.q.rows(); ++i){
	 torque_commands.effort[i] = (desired_joint_states.q(i) - joint_states.q(i))*Kp[i]
	 + (desired_joint_states.qdot(i) - joint_states.qdot(i))*Kd[i]
	 + (desired_joint_states.q(i) - joint_states.q(i))*Ki[i];
      }
      // Publish torque commands.
      torque_commands_pub.publish(torque_commands);
   }
}

CLASS_LOADER_REGISTER_CLASS(PIDjointSpace, rclcpp::Node);