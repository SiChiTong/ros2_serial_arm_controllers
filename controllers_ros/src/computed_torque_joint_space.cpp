#include </home/dan/ros2_my_code/src/ros_serial_arm_controllers/controllers_ros/include/controllers_ros/computed_torque_joint_space.h>

ComputedTorqueJointSpace::ComputedTorqueJointSpace(): rclcpp::Node("computed_torque_control"), current_time(0.0), duration(0.0)
{
   // Get KDL chain.
   ros_tools::getChainFromURDF(kdl_chain);
   
   // Set gravity.
   grav_vec.Zero();
   grav_vec(2)=-9.81;
   
   // Resize.
   Kp.resize(kdl_chain.getNrOfJoints());
   Kd.resize(kdl_chain.getNrOfJoints());
   joint_vel_lims.resize(kdl_chain.getNrOfJoints());
   joint_acc_lims.resize(kdl_chain.getNrOfJoints());
   joint_states_msg.position.resize(kdl_chain.getNrOfJoints());
   joint_states_msg.velocity.resize(kdl_chain.getNrOfJoints());
   joint_start_pos.resize(kdl_chain.getNrOfJoints());
   joint_final_pos.resize(kdl_chain.getNrOfJoints());
   joint_states.q.resize(kdl_chain.getNrOfJoints());
   joint_states.qdot.resize(kdl_chain.getNrOfJoints());
   desired_joint_states.q.resize(kdl_chain.getNrOfJoints());
   desired_joint_states.qdot.resize(kdl_chain.getNrOfJoints());
   desired_joint_states.qdotdot.resize(kdl_chain.getNrOfJoints());
   torque_commands.resize(kdl_chain.getNrOfJoints());
   PID_commands.resize(kdl_chain.getNrOfJoints());
   dynam_commands.resize(kdl_chain.getNrOfJoints());
   mass_mat.resize(kdl_chain.getNrOfJoints());
   gravity.resize(kdl_chain.getNrOfJoints());
   coriolis.resize(kdl_chain.getNrOfJoints());
   torque_commands_msg.effort.resize(kdl_chain.getNrOfJoints());
   torque_commands.resize(kdl_chain.getNrOfJoints());
   
   // Parse parameters. 
   rtt_ros_tools::getJointSpacePgains(kdl_chain, Kp);
   rtt_ros_tools::getJointSpaceDgains(kdl_chain, Kd);
   rtt_ros_tools::getIKsolver(ik_solver);
   rtt_ros_tools::getJointStartPose(kdl_chain, joint_start_pos);
   rtt_ros_tools::getJointSpaceVelProfileName(vel_prof_name);
   joint_space_vel_prof::getJointSpaceVelProfile(vel_prof_name, kdl_chain, vel_prof);
   rtt_ros_tools::transformWaypointsToNewFrame(transform_points);
   if(transform_points) rtt_ros_tools::getFrameID(frame_id);
   
   // Create dynamics solver.
   chain_dynamic_params.reset(new KDL::ChainDynParam(kdl_chain, grav_vec));
}

// Run update loop.
void ComputedTorqueJointSpace::on_new_trajectory(sensor_msgs::joint_states joint_states_msg, geometry_msgs::poseArray trajectory_goal_pose)
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
      ik_solver->CartToJnt(joint_states.q, trajectory_goal_frame, joint_final_pos);
      
      old_timestamp = trajectory_goal_pose.header.stamp;
   }
   // Set to start position if there are no new goal poses.
   else joint_final_pos.data = joint_start_pos.data;
   
   // Set up velocity profile.
   vel_prof->solve(joint_states.q, joint_final_pos, duration);
	 
   // Reset time reference before beginning new trajectory.
   start_time = rclcpp::Time::Now();
   
   while(current_time <= duration){
      
      // Get mass matrix
      chain_dynamic_params->JntToMass(joint_states.q, mass_mat);
      
      // Get gravity forces.
      chain_dynamic_params->JntToGravity(joint_states.q, gravity);
      
      // Get coriolis forces.
      chain_dynamic_params->JntToCoriolis(joint_states.q, joint_states.qdot, coriolis);
      
      // Get desired joint positions.
      vel_prof->get_desired_joint_pos(desired_joint_states.q, current_time);
      
      // Get desired joint velocities.
      vel_prof->get_desired_joint_vel(desired_joint_states.qdot, current_time);
      
      // Get desired joint accelerations.
      vel_prof->get_desired_joint_acc(desired_joint_states.qdotdot, current_time);
      
      // Increase timer.
      current_time = rclcpp::Time::Now() - start_time;
      
      // Solve for torque commands.
      for(int i = 0; i < desired_joint_states.q.rows(); ++i)
      {
	 PID_commands(i) = (desired_joint_states.q(i) - joint_states.q(i))*Kp[i] +
	 (desired_joint_states.qdot(i) - joint_states.qdot(i))*Kd[i] +
	 desired_joint_states.qdotdot(i);
	 
	 dynam_commands(i) = coriolis(i)*(desired_joint_states.qdot(i) - joint_states.qdot(i)) + gravity(i);
      }
      torque_commands = mass_mat.data * PID_commands.data + dynam_commands.data;
      
      // Create torque commands message.
      for(int i=0; i<torque_commands.rows(); ++i)
	 torque_commands_msg.effort[i] = torque_commands[i];
      
      // Write torque commands.
      torque_commands_pub.publish(torque_commands_msg);
   }
}

CLASS_LOADER_REGISTER_CLASS(ComputedTorqueJointSpace, rclcpp::Node);
