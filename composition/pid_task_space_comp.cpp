#include <memory>
#include "rclcpp/rclcpp.hpp"

#include <controllers_ros/pid_task_space.h>
#include <ros_gazebo_interface/ros_gazebo_interface.h>
#include <ros_trajectory_generators/random_waypoint_array_generator.h>
#include <ros_trajectory_generators/cartesian_trajectory_generator.h>


int main(int argc, char* argv[])
{
   rclcpp::init(argc, argv);
   
   // Create an executor that will be responsible for execution of callbacks for a set of nodes.
   rclcpp::executors::SingleThreadedExecutor exec;
   
   // Add some nodes to the executor which provide work for the executor during its "spin" function.
   auto waypoint_generator = std::make_shared<random_waypoint_array_generator>();
   exec.add_node(waypoint_generator);

   auto trajectory_generator = std::make_shared<cartesian_trajectory_generator>();
   exec.add_node(trajectory_generator);
   
   auto controller = std::make_shared<pid_task_space>();
   exec.add_node(controller);
   
   auto gazebo = std::make_shared<ros_gazebo_interface>();
   exec.add_node(gazebo);
   
   // spin will block until work comes in, execute work as it becomes available, and keep blocking.
   // It will only be interrupted by Ctrl-C.
   exec.spin();
   
   rclcpp::shutdown();
   
   return 0;
}
