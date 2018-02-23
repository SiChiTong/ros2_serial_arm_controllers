#include <ros_trajectory_generators/cartesian_trajectory_generator.h>

CartesianTrajectory::CartesianTrajectory(): rclcpp::Node("cartesian_trajectory")
{
   // Parse parameters.
}

void CartesianTrajectory::update(const geometry_msgs::PoseArray& waypoint_array)
{
   if(!trajectory_computed) computeTrajectory();
   
   // If trajectory is computed, run the trajectory.
   while(trajectory_computed){

      if(current_trajec_time < comp_trajec->Duration()){
	 
	 // Get desired states for the current time.
	 current_pos = comp_trajec->Pos(current_trajec_time);
	 current_vel = comp_trajec->Vel(current_trajec_time);
	 current_acc = comp_trajec->Acc(current_trajec_time);
	 
	 tf::poseKDLToMsg(current_pos, cart_trajectory_msg.position.Pose);
	 tf::twistKDLToMsg(current_vel, cart_trajectory_msg.velocity.Twist);
	 tf::twistKDLToMsg(current_acc, cart_trajectory_msg.acceleration.Twist);
	 
	 // Update timer.
	 current_trajec_time = rclcpp::Clock::now() - start_time;
	 
	 // Set timestamp.
	 cart_trajectory_msg.time = current_trajec_time;
	 
	 // Publish message.
	 cart_trajectory.publish(cart_trajectory_msg);
      }
      // Signal that the trajectory is finished.
      else trajectory_computed = false; 
   }
}
   
bool CartesianTrajectory::computeTrajectory(){

   if(transform_points){
      
      waypoint_array.header.frame_id = frame_id;
      pose_stamped.header = waypoint_array.header;
      
      try{
	 for(int i=0; i<waypoint_array.poses.size(); ++i){
	    
	    pose_stamped.pose = waypoint_array.poses[i];
	    tf_listener.transformPose(frame_id, pose_stamped, pose_stamped);
	    waypoint_array.poses[i] = pose_stamped.pose;
	 }
      }catch(tf::TransformException tf_exception){

	 std::cout << tf_exception.what() << std::endl;

	 return false;
      }
   }
   try{
      // Rotates a frame over the existing single rotation axis formed by start and end rotation. If more than one rotational axis exist, an arbitrary one will be choosen.
      interpolator = new KDL::RotationalInterpolation_SingleAxis();
      
      // Trajectory_Composite implements a trajectory that is composed of underlying trajectoria. Call Add to add a trajectory.
      comp_trajec = new KDL::Trajectory_Composite();
      
      // The specification of a path, composed of way-points with rounded corners.
      path = new KDL::Path_RoundedComposite(task_trajectory_corner_radius, task_trajectory_equivalent_radius, interpolator);
      
      // Trapezoidal vel profile constructed from max_vel and max_acc.
      vel_profile = new KDL::VelocityProfile_Trap(task_space_vel_limit, task_space_acc_limit);
      
      // Convert waypoint array to vector of frames.
      for(int i=0; i<waypoint_array.poses.size(); i++){
	 
	 tf::poseMsgToKDL(waypoint_array.poses[i], frame);
	 
	 frame_waypoint_vector.push_back(frame);
      }
      if(frame_waypoint_vector.size() > 1){
	
         // Add each frame to the path.
         std::for_each(frame_waypoint_vector.begin(), frame_waypoint_vector.end(), [&](KDL::Frame waypoint) {path->Add(waypoint);});
	 
	 // Finish creating the path.
         path->Finish();

         // Configure velocity profile based on trajectory start position and end position.
         vel_profile->SetProfile(0, path->PathLength());

         // Trajectory_Segment combines a VelocityProfile and a Path into a trajectory.
         traject = new KDL::Trajectory_Segment(path, vel_profile);

         // Add trajectory segment to the composite trajectory.
         comp_trajec->Add(traject);
      }
      else comp_trajec->Add(new KDL::Trajectory_Segment(new KDL::Path_Point(frame), vel_profile));

      // Wait 0.5s at the end of the trajectory.
      comp_trajec->Add(new KDL::Trajectory_Stationary(0.5, frame));
   }
   // Catch errors.
   catch(KDL::Error& error){

      std::cout << "Planning was attempted with waypoints: " << std::endl;

      for(auto const& point : frame_waypoint_vector)
	 std::cout << point << std::endl;
      
      std::cout <<  error.Description() << std::endl;
      std::cout <<  error.GetType() << std::endl;

      return false;
   }
   // Set time reference for new trajectory.
   start_time = rclcpp::Clock::now();
   
   // Tell update loop that a trajectory is computed.
   trajectory_computed = true;
   
   return true;
}

