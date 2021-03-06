#include </home/dan/ros2_my_code/src/ros_serial_arm_controllers/controllers_ros/include/controllers_ros/operational_space_control.h>

OperationalSpaceControl::OperationalSpaceControl(): rclcpp::Node("operational_space_control")
{
   // Parse parameters.
   ros_tools::getChainFromURDF(kdl_chain);
   ros_tools::getTaskSpacePgains(Kp);
   ros_tools::getTaskSpaceIgains(Ki);
   ros_tools::getTaskSpaceDgains(Kd);
   ros_tools::getEndEffectorFrameName(ee_frame_name);
   ros_tools::getSegmentIndex(ee_frame_index, ee_frame_name, kdl_chain);
   ros_tools::getJointStartPose(kdl_chain, joint_start_pose);
   
   // Set gravity
   grav_vec.Zero();
   grav_vec(2)=-9.81; 
   
   // Create solvers.
   chain_dynamic_params.reset(new KDL::ChainDynParam(kdl_chain, grav_vec));
   jacobian_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain));
   fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));
   fk_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain));
   fk_acc_solver.reset(new KDL::ChainFkSolverAcc_recursive(kdl_chain));
   
   // Find end effector start pose from joint start pose array.
   fk_pos_solver->JntToCart(joint_start_pose, ee_start_pose, ee_frame_index);
   
   // Convert KDL frame to pose message.
   tf::poseKDLToMsg(ee_start_pose, ee_start_pose_msg);
   
   // Push pose message into pose array for port compatibility.
   ee_start_pose_to_port.poses.push_back(ee_start_pose_msg);
   
   // Resize.
   torque_commands.resize(kdl_chain.getNrOfJoints());
   torque_commands_msg.effort.resize(kdl_chain.getNrOfJoints());
   joint_states.position.resize(kdl_chain.getNrOfJoints());
   joint_states.velocity.resize(kdl_chain.getNrOfJoints());
   joint_pos.resize(kdl_chain.getNrOfJoints());
   joint_vel_array.q.resize(kdl_chain.getNrOfJoints());
   joint_vel_array.qdot.resize(kdl_chain.getNrOfJoints());
   last_joint_vel_array.qdot.data.setZero(kdl_chain.getNrOfJoints());
   joint_acc_array.q.resize(kdl_chain.getNrOfJoints());
   joint_acc_array.qdot.resize(kdl_chain.getNrOfJoints());
   joint_acc_array.qdotdot.resize(kdl_chain.getNrOfJoints());
   joint_space_mass_mat.resize(kdl_chain.getNrOfJoints());
   joint_space_mass_mat_inv.resize(kdl_chain.getNrOfJoints());
   gravity.resize(kdl_chain.getNrOfJoints());
   jacobian.resize(kdl_chain.getNrOfJoints());
}

// Run update loop.
void OperationalSpaceControl::on_new_trajectory()
{
   // If ports have no data, set the robot to its starting position.
   if(inPort_DesiredTaskPos.read(desired_task_pos_msg) == RTT::NewData &&
      inPort_DesiredTaskVel.read(desired_task_vel_msg) == RTT::NewData &&
      inPort_DesiredTaskAcc.read(desired_task_acc_msg) == RTT::NewData){
      
      tf::poseMsgToKDL(desired_task_pos_msg, desired_task_pos);
      tf::twistMsgToKDL(desired_task_vel_msg, desired_task_vel);
      tf::twistMsgToKDL(desired_task_acc_msg, desired_task_acc);
   }
   // Use start position to generate trajectory.
   else outPort_TrajectoryGenerator.write(ee_start_pose_to_port);

   // Read current joint states from robot or Gazebo.
   inPort_CurrentJointStates.read(joint_states);
   
   // Copy data from joint_states to JntArrayVel for compatibility with FK velocity solver.
   joint_vel_array.q.data = Eigen::VectorXd::Map(joint_states.position.data(), joint_states.position.size());
   joint_vel_array.qdot.data = Eigen::VectorXd::Map(joint_states.velocity.data(), joint_states.velocity.size());
   
   // Copy data from joint_vel_array to JntArrayAcc for compatibility with FK acceleration solver.
   joint_acc_array.q.data = joint_vel_array.q.data;
   joint_acc_array.qdot.data = joint_vel_array.qdot.data;
   
   // Use numerical differentiation to find acceleration.
   for(int i=0; i<joint_vel_array.qdot.rows(); ++i)
      joint_acc_array.qdotdot.data[i] = (joint_vel_array.qdot.data[i] - last_joint_vel_array.qdot.data[i]) / this->getPeriod();
   
   last_joint_vel_array.qdot.data = joint_vel_array.qdot.data;
   
   // Get current end effector pose.
   fk_pos_solver->JntToCart(joint_pos, current_task_pos, ee_frame_index);

   // Solve for current end effector velocity.
   fk_vel_solver->JntToCart(joint_vel_array, current_task_vel_solver, ee_frame_index);
   
   // Get current end effector velocity.
   current_task_vel = current_task_vel_solver.GetTwist();

   // Solve for current end effector acceleration.
   fk_acc_solver->JntToCart(joint_acc_array, current_task_acc_solver, ee_frame_index);

   // Get current end effector acceleration;
   current_task_acc = current_task_acc_solver.GetTwist();

   // Get current position error.
   pose_error = diff(desired_task_pos, current_task_pos);

   // Get gravity forces.
   chain_dynamic_params->JntToGravity(joint_pos, gravity);

   // Get joint space mass matrix.
   chain_dynamic_params->JntToMass(joint_pos, joint_space_mass_mat);

   // Get the operational space mass matrix.
   joint_space_mass_mat.data =  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>(joint_space_mass_mat.data.diagonal().asDiagonal());

   joint_space_mass_mat_inv.data = joint_space_mass_mat.data.inverse();

   jacobian_solver->JntToJac(joint_pos, jacobian, ee_frame_index);
   
   op_space_mass_mat = (jacobian.data * joint_space_mass_mat_inv.data * jacobian.data.transpose()).inverse();

   // Convert jacobian to Eigen::Matrix so transpose operation can be used.
   jacobian_transpose = jacobian.data;
   
   // transpose jacobian.
   jacobian_transpose.transposeInPlace();

   // Make a vector of PID commands.
   for(int i = 0; i < 6; ++i)
      PID_commands(i) = pose_error(i)*Kp[i] + (desired_task_vel(i) - current_task_vel(i))*Kd[i]
      + (desired_task_acc(i) - current_task_acc(i))*Ki[i];
   
   // Convert KDL::Twist to Eigen::Matrix so we can use matrix multiplications.
   tf::twistKDLToEigen(PID_commands, pid_commands);
   
   // Get torque commands.
   torque_commands = jacobian_transpose * op_space_mass_mat * pid_commands + gravity.data;
   
   for(int i=0; i<torque_commands.rows(); ++i)
      torque_commands_msg.effort[i] = torque_commands[i];
   
   // Write torque commands to port.
   outPort_TorqueCommands.write(torque_commands_msg);
}

CLASS_LOADER_REGISTER_CLASS(OperationalSpaceControl, rclcpp::Node);
