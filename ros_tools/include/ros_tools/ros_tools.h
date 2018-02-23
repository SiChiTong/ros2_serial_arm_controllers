#ifndef ROS_TOOLS_H
#define ROS_TOOLS_H

#include <ros/ros.h>

#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/velocityprofile.hpp>
#include <kdl/velocityprofile_dirac.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <trac_ik/trac_ik.hpp>
#include <urdf/model.h>

#include <memory>
#include <assert.h>
#include <map>

namespace ros_tools {

   bool getChainFromURDF(KDL::Chain& kdl_chain);
   
   bool getJointPoseLimsFromURDF(std::vector<double>& lower_limits, std::vector<double>& upper_limits, const KDL::Chain& kdl_chain);
   
   bool getJointEffortLimsFromURDF(const KDL::Chain& kdl_chain, std::vector<double>& joint_effort_lims);
   
   bool getJointDynamicsFromURDF(std::vector<double>& friction, std::vector<double>& damping, const KDL::Chain& kdl_chain);
   
   bool getJointVelLimsFromURDF(const KDL::Chain& kdl_chain, std::vector<double>& joint_vel_lims);
   
   bool getJointAccLims(const KDL::Chain& kdl_chain, std::vector<double>& joint_acc_lims);
   
   bool getJointSpacePgains(const KDL::Chain& kdl_chain, std::vector<double>& joint_p_gains);
   
   bool getJointSpaceIgains(const KDL::Chain& kdl_chain, std::vector<double>& joint_i_gains);
   
   bool getJointSpaceDgains(const KDL::Chain& kdl_chain, std::vector<double>& joint_d_gains);
   
   bool getJointSpaceVelProfileName(std::string& vel_prof_name);
   
   bool useJointVelLims(bool& use_vel_lims);
   
   bool useJointAccLims(bool& use_acc_lims);
   
   bool getJointStartPose(const KDL::Chain &kdl_chain, KDL::JntArray& joint_start_pose);
   
   bool getIKsolver(std::unique_ptr<TRAC_IK::TRAC_IK>& IK_solver);
   
   bool getTaskSpaceVelLim(double& vel_lim);
   
   bool getTaskSpaceAccLim(double& acc_lim);
   
   bool getTaskTrajectoryEquivalentRadius(double& eq_radius);
   
   bool getTaskTrajectoryCornerRadius(double& radius);
   
   bool getTaskSpaceVelProfile(KDL::VelocityProfile* vel_prof);
   
   bool getTaskSpacePgains(std::vector<double>& task_p_gains);
   
   bool getTaskSpaceIgains(std::vector<double>& task_i_gains);
   
   bool getTaskSpaceDgains(std::vector<double>& task_d_gains);

   bool getSegmentIndex(int& seg_index, std::string segment_name, const KDL::Chain& kdl_chain);

   bool getChainJointNames(const KDL::Chain& kdl_chain, std::vector<std::string>& joint_names);

   bool getRootLink(std::string& root_link);

   bool getTipLink(std::string& tip_link);

   bool getEndEffectorFrameName(std::string& frame_name);
   
   bool transformWaypointsToNewFrame(bool& transform);
   
   bool getFrameID(std::string& frame_id);
   
   bool getGazeboWorldFile(std::string& gz_world);
   
   bool getRobotDescription(std::string& robot_description);
}

#endif
