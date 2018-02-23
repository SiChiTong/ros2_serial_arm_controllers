#include </home/dan/ros2_my_code/src/ros_serial_arm_controllers/controllers_ros/include/controllers_ros/pid_task_space.h>

PIDtaskSpace::PIDtaskSpace(): rclcpp::Node("pid_task_space")
{
   // Parse parameters.
   ros_tools::getChainFromURDF(kdl_chain);
   ros_tools::getTaskSpacePgains(Kp);
   ros_tools::getTaskSpaceIgains(Ki);
   ros_tools::getTaskSpaceDgains(Kd);
   ros_tools::getJointStartPose(kdl_chain, joint_start_pos);
   ros_tools::getEndEffectorFrameName(ee_frame_name);
   ros_tools::getSegmentIndex(ee_frame_index, ee_frame_name, kdl_chain);
   
   // Create solvers.
   fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));
   fk_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain));
   jacobian_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain));
   
   // Find end effector start pose from joint start pose array.
   fk_pos_solver->JntToCart(joint_start_pos, ee_start_pose_frame, ee_frame_index);
   
   // Convert KDL frame to pose message.
   tf::poseKDLToMsg(ee_start_pose_frame, ee_start_pose_msg);
   
   // Push pose message into pose array for port compatibility.
   ee_start_pose_to_port.poses.push_back(ee_start_pose_msg);
   
   // Resize.
   Kp.resize(kdl_chain.getNrOfJoints());
   Ki.resize(kdl_chain.getNrOfJoints());
   Kd.resize(kdl_chain.getNrOfJoints());
   joint_states_msg.position.resize(kdl_chain.getNrOfJoints());
   joint_states_msg.velocity.resize(kdl_chain.getNrOfJoints());
   joint_states.q.resize(kdl_chain.getNrOfJoints());
   joint_states.qdot.resize(kdl_chain.getNrOfJoints());
   joint_start_pos.resize(kdl_chain.getNrOfJoints());
   torque_commands_msg.effort.resize(kdl_chain.getNrOfJoints());
   torque_commands.resize(kdl_chain.getNrOfJoints());
   jacobian.resize(kdl_chain.getNrOfJoints());
   jacobian_transpose(jacobian.rows(), jacobian.columns());
}

// Run update loop.
void PIDtaskSpace::on_new_trajectory(desired_task_pos_msg, desired_task_vel_msg, joint_states_msg)
{
   if(desired_task_pos_msg = nullptr)
      trajectory_generator_pub.publish(ee_start_pose_to_port);
   
   tf::poseMsgToKDL(desired_task_pos_msg, desired_task_pos);
   tf::twistMsgToKDL(desired_task_vel_msg, desired_task_vel);

   // Move data from messages to JntArrayVel for compatibility with fk velocity solvers.
   joint_states.q.data = Eigen::VectorXd::Map(joint_states_msg.position.data(), joint_states_msg.position.size());
   joint_states.qdot.data = Eigen::VectorXd::Map(joint_states_msg.velocity.data(), joint_states_msg.velocity.size());

   // Get current end effector pose.
   fk_pos_solver->JntToCart(joint_states.q, current_task_pos, ee_frame_index);

   // Solve for current end effector velocity.
   fk_vel_solver->JntToCart(joint_states, current_task_vel_solver, ee_frame_index);

   // Get current end effector velocity.
   current_task_vel = current_task_vel_solver.GetTwist();

   // Get current position error.
   pose_error = diff(desired_task_pos, current_task_pos);

   // Make a vector of PID commands.
   for(int i = 0; i < 6; ++i)
      PID_commands(i) = pose_error(i)*Kp[i] + pose_error(i)*Ki[i] + (desired_task_vel(i) - current_task_vel(i))*Kd[i];

   // Convert KDL::Twist to Eigen::Matrix so we can use matrix multiplications.
   tf::twistKDLToEigen(PID_commands, pid_commands);

   // Get jacobian.
   jacobian_solver->JntToJac(joint_states.q, jacobian, ee_frame_index);

   // Convert jacobian to Eigen::Matrix so transpose operation can be used.
   jacobian_transpose = jacobian.data;

   // transpose jacobian.
   jacobian_transpose.transposeInPlace();

   // Get torque commands.
   torque_commands = jacobian_transpose * pid_commands;

   for(int i=0; i<torque_commands.rows(); ++i)
      torque_commands_msg.effort[i] = torque_commands[i];
   
   torque_commands_msg.header.stamp = rclcpp::Clock::now();

   // Write torque commands to port.
   torque_commands_pub.publish(torque_commands_msg);
}

CLASS_LOADER_REGISTER_CLASS(PIDtaskSpace, rclcpp::Node);