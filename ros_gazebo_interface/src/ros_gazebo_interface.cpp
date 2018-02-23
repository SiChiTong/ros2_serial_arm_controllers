#include </home/dan/ros2_my_code/src/ros_serial_arm_controllers/ros_gazebo_interface/include/ros_gazebo_interface/ros_gazebo_interface.h>

GazeboInterface::GazeboInterface(): rclcpp::Node("gazebo_interface")
{
   // Parse parameters.
   ros_tools::getGazeboWorldFile(gazebo_world);
   ros_tools::getRobotDescription(robot_description);
   ros_tools::getChainFromURDF(kdl_chain);
   
   // Resize.
   joint_states.position.resize(kdl_chain.getNrOfJoints());
   joint_states.velocity.resize(kdl_chain.getNrOfJoints());
   torque_commands.effort.resize(kdl_chain.getNrOfJoints());
   
   auto command_callback = [this](control_msgs::TorqueCommands torque_commands){
      std::make_shared<control_msgs::TorqueCommands>(torque_commands);
   };
   torque_commands_sub = this->create_subscription<control_msgs::TorqueCommands>("torque_commands", &command_callback);
}

bool GazeboInterface::startGazebo()
{
   // No command line arguments are needed.
   argv.clear();
   
   // Set up Gazebo server.
   if(!gazebo::setupServer(argv)){
      ROS_ERROR_STREAM("Could not start Gazebo server!");
      return false;
   }
   // Attempt to load world.
   loaded_world = gazebo::loadWorld(gazebo_world);
   
   // Check if world is loaded.
   if(loaded_world.get() == nullptr) 
      ROS_ERROR_STREAM("Failed to load Gazebo world!");
   
   else ROS_INFO_STREAM("Loaded Gazebo world: " << loaded_world->Name());
   
   // Spawn model from robot_description.
   spawnModel(robot_description);
   
   // Get model's joints from Gazebo.
   gazebo_joints = model->GetJoints();
   
   // Get joint names from Gazebo.
   for(auto joint = gazebo_joints.begin(); joint != gazebo_joints.end(); ++joint)
      gazebo_joint_names.push_back((*joint)->GetName());
   
   // Name of first joint in KDL chain.
   first_joint = kdl_chain.getSegment(0).getJoint().getName();
   
   // Name of last joint in KDL chain.
   last_joint = kdl_chain.getSegment(kdl_chain.getNrOfSegments()-1).getJoint().getName();
   
   // Find the joint index in Gazebo that correstponds to the first joint in the KDL chain.
   first_joint_index = std::distance(gazebo_joint_names.begin(), std::find(gazebo_joint_names.begin(), gazebo_joint_names.end(), first_joint));
   
   ROS_INFO_STREAM("Index of chain's first joint in Gazebo: " << first_joint_index);
   
   // Find the joint index in Gazebo that correstponds to the last joint in the KDL chain.
   last_joint_index = std::distance(gazebo_joint_names.begin(), std::find(gazebo_joint_names.begin(), gazebo_joint_names.end(), last_joint));
   
   ROS_INFO_STREAM("Index of chain's last joint in Gazebo: " << last_joint_index);
   
   // Make sure the joint indices are feasable.
   assert(last_joint_index - first_joint_index == kdl_chain.getNrOfJoints());
   
   // Run world.
   gz_thread = std::thread(gazebo::runWorld, loaded_world, 0);
   
   if(!gz_thread.joinable()){
      ROS_ERROR_STREAM("Could not start Gazebo world");
      return false;
   }
   // Connect to Gazebo world update events.
   gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboInterface::WorldUpdateBegin, this));
   gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&GazeboInterface::WorldUpdateEnd, this));
   
   return true;
}

// Called from Gazebo.
void GazeboInterface::WorldUpdateBegin()
{
   // Run sensors
   gazebo::sensors::run_once();
   
   // Write torque commands to Gazebo.
   for(int i = first_joint_index; i<last_joint_index; ++i)
      gazebo_joints[i]->SetForce(0, torque_commands.effort[i]);
}

// Called from Gazebo.
void GazeboInterface::WorldUpdateEnd()
{
   // Get joint states from Gazebo.
   // Two loops because Gazebo and KDL chain may have different joint indices.
   for(int i = first_joint_index; i<last_joint_index; ++i){
      for(int c = 0; c < kdl_chain.getNrOfJoints(); ++c){
         joint_states.position[c] =  gazebo_joints[i]->Position(0);
         joint_states.velocity[c] = gazebo_joints[i]->GetVelocity(0);
      }
   }
   // Publish joint states.
   joint_states_pub.publish(joint_states);
}

// Add plugins.
void GazeboInterface::addPlugin(const std::string& filename)
{
   gazebo::addPlugin(filename);
}

// Spawn model.
bool GazeboInterface::spawnModel(const std::string& model_description)
{
   // Model count before new model is added.
   int modelCountBeforeSpawn = loaded_world->ModelCount();

   ROS_INFO_STREAM("Spawning model...");
   
   // Attempt to load the model.
   loaded_world->InsertModelString(model_description);
   
   // Count load attempts.
   int load_attempt = 0;
   
   // While the model has not shown up in Gazebo..
   while(loaded_world->ModelCount() != modelCountBeforeSpawn+1){
      
      // Need to advange the physics engine in order for the model count to increase. Run world for 1 iteration.
      gazebo::runWorld(loaded_world,1);
      
      // Sleep for 0.5 sec.
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      
      // Increase counter and check count.
      if(load_attempt++ > 10){
	 ROS_ERROR_STREAM("Failed to spawn model!");
	 return false;
      }
   }
   ROS_INFO_STREAM("Successfuly spawned model: " << model.get()->GetName());
   return true;
}

// Reset world.
void GazeboInterface::resetWorld()
{
   ROS_INFO_STREAM("Resetting Gazebo world.");

   loaded_world->Reset();
}

GazeboInterface::~GazeboInterface()
{
   ROS_INFO_STREAM("Stoping Simulation.");

   gazebo::event::Events::sigInt.Signal();

   loaded_world->Fini();
   
   ROS_INFO_STREAM("Shutting down Gazebo");

   gazebo::shutdown();
}

CLASS_LOADER_REGISTER_CLASS(GazeboInterface, rclcpp::Node);
