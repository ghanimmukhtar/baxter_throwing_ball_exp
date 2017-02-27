#ifndef __LIB_MOVEMENT_H__
#define __LIB_MOVEMENT_H__

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>
#include <Eigen/Core>
#include <boost/timer.hpp>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionFeedback.h>
#include <actionlib/client/simple_action_client.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>

#include "../../src/parameters.hpp"


/**
 * @brief Execute a trajectory given as joint trajectory using joint action server
 * @param action client to joint action server
 * @param joint trajectory_msgs
 * @return true if the guiding of the arm is successful, false otherwise
**/
bool execute_joint_trajectory(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac,
                              trajectory_msgs::JointTrajectory& joint_trajectory,
                              Data_config& parameters,
                              std::ofstream &output_file);

bool go_to_initial_position(Data_config& parameters, actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &ac);


std::vector<double>& extract_certain_arm_joints_values(Data_config& parameters, std::string arm);

/**
 * @brief Check if all joint trajectory points are valid (in terms of self collision)
 * @param Data_config class
 * @return true if trajectory is valid, false otherwise
**/
bool is_trajectory_valid(Data_config& parameters);

/**
 * @brief Extract joint values from the sensor_msgs::JointState using the correct arm choise set in parameters
 * @param Data_config class
 * @return vector of joint values and fill in the parameters the corresponding joint values
**/
std::vector<double>& extract_arm_joints_values(Data_config& parameters);

/**
 * @brief Read a text file that include joint waypoints and construct the joint trajectory_msgs
 * @param text file that includes joints waypoints
 * @param Data_config class
 * @return Nothing but fill in the parameters the joint trajectory_msgs to be followed
**/
void record_arm_joint_trajectory(std::ofstream& output_file, Data_config& parameters);

/**
 * @brief largest_difference
 * @param first
 * @param second
 * @return double that represents the largest difference between the two vectors
 */
double largest_difference(std::vector<double> &first, std::vector<double> &second);

/**
 * @brief Read a vector that includes joint waypoints and construct the joint trajectory_msgs
 * @param the trajectory to be filled
 * @param the vector of waypoints
 * @param double that represents the delta time between points
 * @return Nothing but fill in the parameters the joint trajectory_msgs to be followed
**/
void construct_joint_trajectory_from_vector(trajectory_msgs::JointTrajectory& my_joint_trajectory,
                                            std::vector<std::vector<double> >& raw_joint_traj, double& dt);

/**
 * @brief Read a text file that includes joint waypoints and construct the joint trajectory_msgs
 * @param text file that includes joints waypoints
 * @param Data_config class
 * @return Nothing but fill in the parameters the joint trajectory_msgs to be followed
**/
void construct_joint_trajectory_from_file(std::ifstream &text_file, Data_config& parameters);

#endif /* __LIB_MOVEMENT_H__ */

