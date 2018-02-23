#include <ros_tools/ros_tools.h>

namespace ros_tools{

   bool getChainFromURDF(KDL::Chain& kdl_chain)
   {
      std::string root_link, tip_link, robot_description;
      
      if(!ros::param::get("/root_link", root_link)){
         ROS_ERROR_STREAM("Could not find parameter \"root_link\". (string)");
         return false;
      }
      if(!ros::param::get("/tip_link", tip_link)){
         ROS_ERROR_STREAM("Could not find parameter \"tip_link\". (string)");
         return false;
      }
      if(!ros::param::get("/robot_description", robot_description)){
         ROS_ERROR_STREAM("Could not find parameter \"robot_description\". (string)");
         return false;
      }
      KDL::Tree kdl_tree;
      
      if(!kdl_parser::treeFromString(root_link, kdl_tree)){
         ROS_ERROR_STREAM("Failed to construct a KDL tree.");
         return false;
      }
      const KDL::SegmentMap& segments(kdl_tree.getSegments());
      ROS_INFO_STREAM("Created a KDL tree with segments:");
      for(auto & seg : segments) ROS_INFO_STREAM(seg.first);

      if(segments.find(root_link) == segments.end()){
	 ROS_ERROR_STREAM("root_link '" << root_link << "' was not found in the KDL tree.");
         return false;
      }
      if(segments.find(tip_link) == segments.end()){
	 ROS_ERROR_STREAM("tip_link '" << tip_link << "' was not found in the KDL tree.");
         return false;
      }
      if(!kdl_tree.getChain(root_link, tip_link, kdl_chain)){
         ROS_ERROR_STREAM("Could not build a KDL chain with root_link: " << root_link << " and tip_link: " << tip_link);
         return false;
      }
      ROS_INFO_STREAM("Building a KDL chain from " << root_link << " to " << tip_link);
      ROS_INFO_STREAM("chain has " << kdl_chain.getNrOfJoints() << " joints and " << kdl_chain.getNrOfSegments() << " segments.");
      ROS_INFO_STREAM("Segments Names:");
      for(int i=0; i<kdl_chain.getNrOfSegments(); ++i)
      ROS_INFO_STREAM(kdl_chain.getSegment(i).getName());
      ROS_INFO_STREAM("Joint Names:");
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i)
      ROS_INFO_STREAM(kdl_chain.getSegment(i).getJoint().getName());
      return true;
   }

   bool getGazeboWorldFile(std::string& gz_world)
   {
      if(!ros::param::get("/gazebo_world_path", gz_world)){
         ROS_WARN_STREAM("Could not find parameter \"gazebo_world_path\". Using empty world.");
	 gz_world = "worlds/empty.world";
      }
      else ROS_INFO_STREAM("Found parameter \"gazebo_world_path\": " << gz_world);
      
      std::ifstream worldfile(gz_world);
      
      if(worldfile){
	 ROS_INFO_STREAM("Found world file on gazebo_world_path.");
	 return true;
      }
      else ROS_ERROR_STREAM("gazebo_world_path is not a valid file path!");
      return false;
   }

   bool getRobotDescription(std::string& robot_description)
   {
      if(!ros::param::get("/robot_description", robot_description)){
         ROS_ERROR_STREAM("Could not find parameter \"robot_description\". (string)");
         return false;
      }
      ROS_INFO_STREAM("Found parameter \"robot_description\".");
      return true;
   }

   bool getSegmentIndex(int& seg_index, std::string segment_name, const KDL::Chain& kdl_chain)
   {
      std::map<std::string, int> numbered_segs;
      
      for(int i = 0; i<kdl_chain.getNrOfSegments(); ++i)
         numbered_segs.insert(std::make_pair(kdl_chain.getSegment(i).getName(), i));
      
      auto search = numbered_segs.find(segment_name);
      
      if(search == numbered_segs.end()){
          ROS_ERROR_STREAM("Segment '" << segment_name << "' was not found in the KDL chain.");
         return false;
      }
      ROS_INFO_STREAM("Found chain segment '" << search->first << "' with index '" << search->second << "'");
      seg_index = search->second;
      return true;
   }

   bool getChainJointNames(const KDL::Chain& kdl_chain, std::vector<std::string>& joint_names)
   {
      joint_names.clear();
      joint_names.reserve(kdl_chain.getNrOfJoints());
     
      for(int i = 0; i<kdl_chain.getNrOfJoints(); ++i)
         joint_names.push_back(kdl_chain.getSegment(i).getJoint().getName());
      
      ROS_INFO_STREAM("Created a vector of joint names: ");
      for(auto & joint : joint_names) ROS_INFO_STREAM(joint);
      return true;
   }

   bool getRootLink(std::string& root_link)
   {
      if(!ros::param::get("/root_link",  root_link)){
          ROS_ERROR_STREAM("Could not find parameter \"root_link\". (string)");
         return false;
      }
      ROS_INFO_STREAM("Found parameter \"root_link\": " << root_link);
      return true;
   }

   bool getTipLink(std::string& tip_link)
   {
      if(!ros::param::get("/tip_link", tip_link)){
          ROS_ERROR_STREAM("Could not find parameter \"tip_link\". (string)");
         return false;
      }
      ROS_INFO_STREAM("Found parameter \"tip_link\": " << tip_link);
      return true;
   }

   bool getEndEffectorFrameName(std::string& frame_name)
   {
      if(!ros::param::get("/end_effector_frame_name", frame_name)){
          ROS_ERROR_STREAM("Could not find parameter \"end_effector_frame_name\". (string)");
         return false;
      }
      ROS_INFO_STREAM("Found parameter \"end_effector_frame_name\": " << frame_name);
      return true;
   }

   bool getFrameID(std::string& frame_id, ros::NodeHandle& nh)
   {
      if(!nh.getParam("frame_id", frame_id)){
	 ROS_ERROR_STREAM("Could not find parameter \"frame_id\" (string) for namespace: " << nh.getNamespace());
         return false;
      }
      ROS_INFO_STREAM("Found parameter \"frame_id\": " << frame_id);
      return true;
   }
   
   bool transformWaypointsToNewFrame(bool& transform, ros::NodeHandle& nh)
   {
      if(!nh.getParam("tranform_waypoints_to_new_frame", transform)){
	 ROS_ERROR_STREAM("Could not find parameter \"tranform_waypoints_to_new_frame\" (bool) for node: " << nh.getNamespace());
	 return false;
      }
      ROS_INFO_STREAM("Found parameter \"tranform_waypoints_to_new_frame\": " << transform);
      return true;
   }
     
   bool getJointPoseLimsFromURDF(std::vector<double>& lower_limits, std::vector<double>& upper_limits, const KDL::Chain& kdl_chain)
   {
      lower_limits.clear();
      upper_limits.clear();
      lower_limits.reserve(kdl_chain.getNrOfJoints());
      upper_limits.reserve(kdl_chain.getNrOfJoints());
      
      std::string robot_description;
      double noLimits = 0.0;

      if(!ros::param::get("/robot_description", robot_description)){
         ROS_ERROR_STREAM("Could not find parameter \"robot_description\". (string)");
         return false;
      }
      urdf::Model model;
      boost::shared_ptr<const urdf::Joint> joint;

      if(!model.initString(robot_description)){
         ROS_ERROR_STREAM("Could not create model from \"robot_description\".");
         return false;
      }
      for(int i=0; i<kdl_chain.getNrOfSegments(); ++i)
      {
         joint = model.getJoint(kdl_chain.getSegment(i).getJoint().getName());

         if(joint->safety && joint->limits) {
            lower_limits.push_back(std::max(joint->limits->lower, joint->safety->soft_lower_limit));
            upper_limits.push_back(std::min(joint->limits->upper, joint->safety->soft_upper_limit));
         }
         else if(!joint->safety && joint->limits)
         {
            lower_limits.push_back(joint->limits->lower);
            upper_limits.push_back(joint->limits->upper);
         }
         else if(joint->safety && !joint->limits)
         {
            lower_limits.push_back(joint->safety->soft_lower_limit);
            upper_limits.push_back(joint->safety->soft_upper_limit);
         }
         else
         {
            lower_limits.push_back(noLimits);
            upper_limits.push_back(noLimits);
         }
      }
      ROS_INFO_STREAM("Created vectors of joint pose limits:");
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i){
         ROS_INFO_STREAM(kdl_chain.getSegment(i).getJoint().getName() << ": lower limit " << lower_limits[i] << ", upper limit " << upper_limits[i]);
      }
      return true;
   }

   bool getJointDynamicsFromURDF(std::vector<double>& friction, std::vector<double>& damping, const KDL::Chain& kdl_chain)
   {
      friction.clear();
      damping.clear();
      friction.reserve(kdl_chain.getNrOfJoints());
      damping.reserve(kdl_chain.getNrOfJoints());
  
      std::string robot_description;
      double noLimits = 0.0;

      if(!ros::param::get("robot_description", robot_description)){
         ROS_ERROR_STREAM("Could not find parameter \"robot_description\". (string)");
         return false;
      }
      urdf::Model model;
      boost::shared_ptr<const urdf::Joint> joint;

      if(!model.initString(robot_description)){
          ROS_ERROR_STREAM("Could not create a model from \"robot_description\".");
         return false;
      }
      for(int i=0; i<kdl_chain.getNrOfSegments(); ++i)
      {
         joint = model.getJoint(kdl_chain.getSegment(i).getJoint().getName());

         if(joint->dynamics)
         {
            damping.push_back(joint->dynamics->damping);
            friction.push_back(joint->dynamics->friction);
         }
         else
         {
            damping.push_back(noLimits);
            friction.push_back(noLimits);
         }
      }
      ROS_INFO_STREAM("Created vector of joint friction values:");
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i)
         ROS_INFO_STREAM(kdl_chain.getSegment(i).getJoint().getName() << ": friction value " << friction[i]);
      
      ROS_INFO_STREAM("Created vectors of joint damping values:");
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i)
         ROS_INFO_STREAM(kdl_chain.getSegment(i).getJoint().getName() << ": damping value " << damping[i]);
      return true;
   }

   bool getJointStartPose(const KDL::Chain& kdl_chain, KDL::JntArray& joint_start_pose)
   {
      std::vector<double> joint_start_pose_vec;
      joint_start_pose.resize(kdl_chain.getNrOfJoints());
      
      if(!ros::param::get("/joint_start_pose", joint_start_pose_vec)){
	 ROS_ERROR_STREAM("Could not find parameter \"joint_start_pose\". (array)");
	 return false;
      }
      if(joint_start_pose_vec.size() != kdl_chain.getNrOfJoints()){
         ROS_ERROR_STREAM("Size of array \"joint_start_pose\" must be equal to the number of joints in the KDL chain!");
         return false;
      }
      for(int i=0; i<joint_start_pose.rows(); ++i)
         joint_start_pose(i) = joint_start_pose_vec[i];
      
      ROS_INFO_STREAM("Created a KDL JntArray for joint start poses:");
      for(int i=0; i<joint_start_pose.rows(); ++i)
         ROS_INFO_STREAM(kdl_chain.getSegment(i).getJoint().getName() << ": start angle " << joint_start_pose(i));
      return true;
   }

   bool getJointEffortLimsFromURDF(const KDL::Chain& kdl_chain, std::vector<double>& joint_effort_lims)
   {
      joint_effort_lims.clear();
      joint_effort_lims.reserve(kdl_chain.getNrOfJoints());
      
      std::string robot_description;
      double noLimits = 0.0;
      
      if(!ros::param::get("/robot_description", robot_description)){
         ROS_ERROR_STREAM("Could not find parameter \"robot_description\". (string)");
         return false;
      }
      urdf::Model model;
      boost::shared_ptr<const urdf::Joint> joint;

      if(!model.initString(robot_description)){
         ROS_ERROR_STREAM("Could not create a model from \"robot_description\".");
         return false;
      }
      for(int i=0; i<kdl_chain.getNrOfSegments(); ++i)
      {
         joint = model.getJoint(kdl_chain.getSegment(i).getJoint().getName());

         if(joint->limits->effort)
	    joint_effort_lims.push_back(joint->limits->effort);
	 
         else joint_effort_lims.push_back(noLimits);
      }
      ROS_ERROR_STREAM("Created vector of joint torque limits:");
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i)
         ROS_ERROR_STREAM(kdl_chain.getSegment(i).getJoint().getName() << ": torque limit " << joint_effort_lims[i]);
      return true;
   }

   bool getJointVelLimsFromURDF(const KDL::Chain& kdl_chain, std::vector<double>& joint_vel_lims)
   {
      joint_vel_lims.clear();
      joint_vel_lims.reserve(kdl_chain.getNrOfJoints());
      
      std::string robot_description;

      if(!ros::param::get("/robot_description", robot_description)){
         ROS_ERROR_STREAM("Could not find parameter \"robot_description\". (string)");
         return false;
      }
      urdf::Model model;
      boost::shared_ptr<const urdf::Joint> joint;

      if(!model.initString(robot_description)){
         ROS_ERROR_STREAM("Could not create a model from \"robot_description\".");
         return false;
      }
      for(int i=0; i<kdl_chain.getNrOfSegments(); ++i)
      {
         double noLimits = 0.0;
         joint = model.getJoint(kdl_chain.getSegment(i).getJoint().getName());

         if(joint->limits->velocity)
	    joint_vel_lims.push_back(joint->limits->velocity);
         
         else joint_vel_lims.push_back(noLimits);
      }
      ROS_INFO_STREAM("Created vector of joint velocity limits:");
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i)
         ROS_INFO_STREAM(kdl_chain.getSegment(i).getJoint().getName() << ": velocity limit " << joint_vel_lims[i]);
      return true;
   }

   bool getJointAccLims(const KDL::Chain& kdl_chain, std::vector<double>& joint_accel_lims)
   {
      joint_accel_lims.clear();
      joint_accel_lims.reserve(kdl_chain.getNrOfJoints());
      
      std::vector<double> joint_accel_lims_param;
      
      if(!ros::param::get("/joint_accel_lims", joint_accel_lims_param)){
	 ROS_ERROR_STREAM("Could not find parameter \"joint_accel_lims\". (array)");
         return false;
      }
      if(joint_accel_lims_param.size() != kdl_chain.getNrOfJoints()){
         ROS_ERROR_STREAM("Size of joint_accel_lims must equal the number of joints in the KDL chain.");
         return false;
      }
      ROS_INFO_STREAM("Created a vector of joint acceleration limits:");
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i)
         ROS_INFO_STREAM(kdl_chain.getSegment(i).getJoint().getName() << ": acceleration limit " << joint_accel_lims[i]);
      return true;
   }

   bool useJointVelLims(bool& use_vel_lims)
   {   
      if(!ros::param::get("/use_joint_vel_lims", use_vel_lims)){
         ROS_ERROR_STREAM("Could not find parameter \"use_joint_vel_lims\". (bool)");
         return false;
      }
      ROS_INFO_STREAM("Found param_error \"use_joint_vel_lims\": " << use_vel_lims);
      return true;
   }

   bool useJointAccLims(bool& use_acc_lims)
   {
      if(!ros::param::get("/use_joint_acc_lims", use_acc_lims)){
         ROS_ERROR_STREAM("Could not find parameter \"use_joint_acc_lims\". (bool)");
         return false;
      }
      ROS_INFO_STREAM("Found param_error \"use_joint_acc_lims\": " << use_acc_lims);
      return true;
   }

   bool getTaskSpaceVelLim(double& vel_lim)
   {
      if(!ros::param::get("/task_space_vel_limit", vel_lim)){
         ROS_ERROR_STREAM("Could not find parameter \"task_space_vel_limit\". (double)");
         return false;
      }
      ROS_INFO_STREAM("Found parameter \"task_space_vel_limit\": " << vel_lim << " m/s");
      return true;
   }

   bool getTaskSpaceAccLim(double& acc_lim)
   {
      if(!ros::param::get("/task_space_acc_limit", acc_lim)){
         ROS_ERROR_STREAM("Could not find parameter \"task_space_acc_limit\". (double)");
         return false;
      }
      ROS_INFO_STREAM("Found parameter \"task_space_acc_limit\": " << acc_lim << " m/s^2");
      return true;
   }

   bool getTaskTrajectoryEquivalentRadius(double& eq_radius)
   {
      if(!ros::param::get("/task_trajectory_equivalent_radius", eq_radius)){
         ROS_WARN_STREAM("Could not find parameter \"task_trajectory_equivalent_radius\". Using default value 0.05 m");
         eq_radius = 0.05;
         return true;
      }
      ROS_INFO_STREAM("Found parameter \"task_trajectory_equivalent_radius\": " << eq_radius << " m");
      return true;
   }

   bool getTaskTrajectoryCornerRadius(double& radius)
   {
      if(!ros::param::get("/task_trajectory_corner_radius", radius)){
         ROS_WARN_STREAM("Could not find parameter \"task_trajectory_corner_radius\". Using default value 0.01 m");
         radius = 0.01;
         return true;
      }
      ROS_INFO_STREAM("Found parameter \"task_trajectory_corner_radius\": " << radius << " m");
      return true;
   }

   bool getJointSpaceVelProfileName(std::string& vel_prof_name)
   {
      if(!ros::param::get("/joint_space_velocity_profile", vel_prof_name)){
          ROS_ERROR_STREAM("Could not find parameter \"joint_space_velocity_profile\". (string)");
         return false;
      }
      ROS_INFO_STREAM("Found parameter \"joint_space_velocity_profile\": " << vel_prof_name);
      return true;
   }

   bool getTaskSpaceVelProfile(KDL::VelocityProfile* vel_prof)
   {
      double max_vel, max_acc;
      std::string task_space_velocity_profile;

      if(!ros::param::get("/task_space_velocity_profile", task_space_velocity_profile)){
         ROS_ERROR_STREAM("Could not find parameter \"task_space_velocity_profile\". (string)");
         return false;
      }
      ROS_INFO_STREAM("Found parameter \"task_space_velocity_profile\": " << task_space_velocity_profile);

      if(task_space_velocity_profile == "diracvel"){
         vel_prof = new KDL::VelocityProfile_Dirac();
         return true;
      }
      else if(task_space_velocity_profile == "rectangular"){
	 getTaskSpaceVelLim(max_vel);
         vel_prof = new KDL::VelocityProfile_Rectangular(max_vel);
         return true;
      }
      else if(task_space_velocity_profile == "trapezoidal"){
	 getTaskSpaceVelLim(max_vel);
	 getTaskSpaceAccLim(max_acc);
         vel_prof = new KDL::VelocityProfile_Trap(max_vel, max_acc);
         return true;
      }
      else if(task_space_velocity_profile == "spline"){
	 getTaskSpaceVelLim(max_vel);
	 getTaskSpaceAccLim(max_acc);
         vel_prof = new KDL::VelocityProfile_Trap(max_vel, max_acc);
         return true;
      }
      else{ ROS_INFO_STREAM("function argument" << task_space_velocity_profile << "does not match a velocity profile."
	 "Options are \"diracvel\", \"rectangular\", \"trapezoidal\", \"spline\"");
         return false;
      }
   }

   bool getIKsolver(std::unique_ptr<TRAC_IK::TRAC_IK>& ik_solver)
   {
      bool param_error(false);
      double max_solve_time, error;
      TRAC_IK::SolveType solver;
      std::string solver_type, robot_description, root_link, tip_link;
      
      if(ros::param::get("/solver_type", solver_type)){
	 
	 if(solver_type == "Speed")
	    solver = TRAC_IK::Speed;
	 
	 else if(solver_type == "Manip")
	    solver = TRAC_IK::Manip1;
	 
	 else if(solver_type == "Distance")
	    solver = TRAC_IK::Distance;
	 
	 else{
	    ROS_WARN_STREAM("Could not resolve parameter \"solver_type\". Options are \"Speed\", \"Manip\", \"Distance\".");
	    ROS_WARN_STREAM("Using default solver mode \"Speed\".");
	    solver = TRAC_IK::Speed;
	 }
      }
      else{
	 ROS_WARN_STREAM("Could not find parameter \"solver_type\". Using default solver mode \"Speed\".");
	 solver = TRAC_IK::Speed;
      }
      if(!ros::param::get("/max_solve_time", max_solve_time)){
	 ROS_WARN_STREAM("Could not find parameter \"max_solve_time\". Using default value 0.005 seconds.");
	 max_solve_time = 0.005;
      }
      if(!ros::param::get("/error", error)){
	 ROS_WARN_STREAM("Could not find parameter \"error\". Using default value: 1e-5 m.");
	 error = 0.00001;
      }
      if(!ros::param::get("/robot_description", robot_description)){
	 ROS_ERROR_STREAM("Could not find parameter \"robot_description\". (string)");
	 param_error = true;
      }
      if(!ros::param::get("/root_link",  root_link)){
	 ROS_ERROR_STREAM("Could not find parameter \"root_link\". (string)");
	 param_error = true;
      }
      if(!ros::param::get("/tip_link",  tip_link)){
	 ROS_ERROR_STREAM("Could not find parameter \"tip_link\". (string)");
	 param_error = true;
      }
      if(param_error) return false;
      ik_solver.reset(new TRAC_IK::TRAC_IK(root_link, tip_link, "robot_description", max_solve_time, error, solver));
      ROS_INFO_STREAM("Created Trac-IK inverse kinematics solver with parameters -> root_link: " << root_link << ", tip_link: " << tip_link <<
      ", robot_description, max_solve_time: " << max_solve_time << ", error tolerance: " << error << ", solver mode: " << solver);
      return true;
   }
   
   bool getJointSpacePgains(const KDL::Chain& kdl_chain, std::vector<double>& joint_p_gains)
   {
      joint_p_gains.clear();
      joint_p_gains.reserve(kdl_chain.getNrOfJoints());
      
      if(!ros::param::get("/joint_P_gains", joint_p_gains)){
	 ROS_ERROR_STREAM("Could not find parameter \"joint_P_gains\". (array)");
	 return false;
      }
      if(joint_p_gains.size() != kdl_chain.getNrOfJoints()){
	 ROS_ERROR_STREAM("Size of joint_P_gains must be equal to the number of joints in the KDL chain!");
	 return false;
      }
      ROS_INFO_STREAM("Created vector of joint space proportional gains:");
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i)
	 ROS_INFO_STREAM(kdl_chain.getSegment(i).getJoint().getName() << ": proportional gain " << joint_p_gains[i]);
      return true;
   }

   bool getJointSpaceIgains(const KDL::Chain& kdl_chain, std::vector<double>& joint_i_gains)
   {
      joint_i_gains.clear();
      joint_i_gains.reserve(kdl_chain.getNrOfJoints());
      
      if(!ros::param::get("/joint_I_gains", joint_i_gains)){
	  ROS_ERROR_STREAM("Could not find parameter \"joint_I_gains\". (array)");
	 return false;
      }
      if(joint_i_gains.size() != kdl_chain.getNrOfJoints()){
	 ROS_ERROR_STREAM("Size of joint_I_gains must be equal to the number of joints in the KDL chain!");
	 return false;
      }
      ROS_INFO_STREAM("Created vector of joint space integral gains:");
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i)
	 ROS_INFO_STREAM(kdl_chain.getSegment(i).getJoint().getName() << ": integral gain "<<joint_i_gains[i]);
      return true;
   }

   bool getJointSpaceDgains(const KDL::Chain& kdl_chain, std::vector<double>& joint_d_gains)
   {
      joint_d_gains.clear();
      joint_d_gains.reserve(kdl_chain.getNrOfJoints());
      
      if(!ros::param::get("/joint_D_gains", joint_d_gains)){
	 ROS_ERROR_STREAM("Could not find parameter \"joint_D_gains\". (array)");
	 return false;
      }
      if(joint_d_gains.size() != kdl_chain.getNrOfJoints()){
	 ROS_ERROR_STREAM("Size of joint_D_gains must be equal to the number of joints in the KDL chain!");
	 return false;
      }
      ROS_INFO_STREAM("Created vector of joint space derivative gains:");
      for(int i=0; i<kdl_chain.getNrOfJoints(); ++i)
	 ROS_INFO_STREAM(kdl_chain.getSegment(i).getJoint().getName() << ": derivative gain " << joint_d_gains[i]);
      return true;
   }
   
   bool getTaskSpacePgains(std::vector<double>& task_p_gains)
   {
      task_p_gains.clear();
      task_p_gains.resize(6);
      
      if(!ros::param::get("/task_P_gains", task_p_gains)){
	 ROS_ERROR_STREAM("Could not find parameter \"task_P_gains\". (array)");
	 return false;
      }
      if(task_p_gains.size() != 6){
	 ROS_ERROR_STREAM("task_P_gains array must have 6 elements. (KDL::Twist has 6 DOF)");
	 return false;
      }
      ROS_INFO_STREAM("Created vector of task space proportional gains:");
      ROS_INFO_STREAM("(Vx,Vy,Vz,Wx,Wy,Wz) -> (" << task_p_gains[0] << "," << task_p_gains[1] << "," << task_p_gains[2] << "," << task_p_gains[3]
      << "," << task_p_gains[4] << "," << task_p_gains[5] << ")");
      return true;
   }

   bool getTaskSpaceIgains(std::vector<double>& task_i_gains)
   {
      task_i_gains.clear();
      task_i_gains.resize(6);
      
      if(!ros::param::get("/task_I_gains", task_i_gains)){
	 ROS_ERROR_STREAM("Could not find parameter \"task_I_gains\". (array)");
	 return false;
      }
      if(task_i_gains.size() != 6){
	 ROS_ERROR_STREAM("task_I_gains array must have 6 elements. (KDL::Twist has 6 DOF)");
	 return false;
      }
      ROS_INFO_STREAM("Created vector of task space integral gains:");
      ROS_INFO_STREAM("(Vx,Vy,Vz,Wx,Wy,Wz) -> (" << task_i_gains[0] << "," << task_i_gains[1] << "," << task_i_gains[2] << "," << task_i_gains[3]
      << "," << task_i_gains[4] << "," << task_i_gains[5] << ")");
      return true;
   }
   
   bool getTaskSpaceDgains(std::vector<double>& task_d_gains)
   {
      task_d_gains.clear();
      task_d_gains.resize(6);
      
      if(!ros::param::get("task_D_gains", task_d_gains)){
	 ROS_ERROR_STREAM("Could not find parameter \"task_D_gains\". (array)");
	 return false;
      }
      if(task_d_gains.size() != 6){
	 ROS_ERROR_STREAM("task_D_gains array must have 6 elements. (KDL::Twist has 6 DOF)");
	 return false;
      }
      ROS_INFO_STREAM("Created vector of task space derivative gains:");
      ROS_INFO_STREAM("(Vx,Vy,Vz,Wx,Wy,Wz) -> (" << task_d_gains[0] << "," << task_d_gains[1] << "," << task_d_gains[2] << "," << task_d_gains[3]
      << "," << task_d_gains[4] << "," << task_d_gains[5] << ")");
      return true;
   }
}
