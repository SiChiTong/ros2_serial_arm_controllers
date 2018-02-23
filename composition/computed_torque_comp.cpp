#include <memory>
#include "rclcpp/rclcpp.hpp"

#include <controllers_ros/computed_torque_joint_space.h>
#include <ros_gazebo_interface/ros_gazebo_interface.h>
#include <ros_trajectory_generators/random_waypoint_generator.h>


int main(int argc, char* argv[])
{
   rclcpp::init(argc, argv);
   
   // Create an executor that will be responsible for execution of callbacks for a set of nodes.
   rclcpp::executors::SingleThreadedExecutor exec;
   
   auto waypoint_generator = std::make_shared<random_waypoint_generator>();
   exec.add_node(waypoint_generator);
   
   // Add some nodes to the executor which provide work for the executor during its "spin" function.
   auto controller = std::make_shared<computed_torque_joint_space>();
   exec.add_node(controller);
   
   auto gazebo = std::make_shared<ros_gazebo_interface>();
   exec.add_node(gazebo);
   
   // spin will block until work comes in, execute work as it becomes available, and keep blocking.
   // It will only be interrupted by Ctrl-C.
   exec.spin();
   
   rclcpp::shutdown();
   
   return 0;
}
