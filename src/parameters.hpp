#ifndef __SIM_PARAM_HPP__
#define __SIM_PARAM_HPP__
#include <ros/ros.h>
#include <string>
#include <math.h>

#include <Eigen/Core>
#include <vector>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf/tf.h>

struct Parameters {
    ///////// OBJECT tracking variables
    // object position (x, y, z) in real world in robot base frame
    trajectory_msgs::JointTrajectoryPoint pt;
    trajectory_msgs::JointTrajectory joint_trajectory;
    std::string baxter_arm;
    std::vector<std::string> baxter_right_arm_joints_names = {"right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"};
    std::vector<std::string> baxter_left_arm_joints_names = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
    sensor_msgs::JointState my_joint_state;
    std::vector<double> right_arm_joints, left_arm_joints;
    double dt;
    robot_model::RobotModelPtr robot_model;
};

class Data_config{

public:
    Parameters params;
    /*Data_config(){
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        params.robot_model = robot_model_loader.getModel();
    }*/

    void create_model(){
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        params.robot_model = robot_model_loader.getModel();
    }

    ///getters
    trajectory_msgs::JointTrajectoryPoint& get_joint_trajectory_point(){
        return params.pt;
    }

    trajectory_msgs::JointTrajectory& get_joint_trajectory(){
        return params.joint_trajectory;
    }

    std::string& get_baxter_arm(){
        return params.baxter_arm;
    }

    std::vector<std::string>& get_baxter_arm_joints_names(std::string& arm){
        if(strcmp(arm.c_str(), "right_arm") == 0)
            return params.baxter_right_arm_joints_names;
        else
            return params.baxter_left_arm_joints_names;
    }

    sensor_msgs::JointState& get_joint_state(){
        return params.my_joint_state;
    }

    std::vector<double>& get_baxter_arm_joint_values(std::string& arm){
        if(strcmp(arm.c_str(), "right_arm") == 0)
            return params.right_arm_joints;
        else
            return params.left_arm_joints;
    }

    double& get_dt(){
        return params.dt;
    }

    robot_model::RobotModelPtr& get_baxter_robot_model(){
        return params.robot_model;
    }

    ///setters
    void set_joint_traj_point(trajectory_msgs::JointTrajectoryPoint& my_joint_traj_point){
        params.pt = my_joint_traj_point;
    }

    void set_joint_trajectory(trajectory_msgs::JointTrajectory& my_joint_traj){
        params.joint_trajectory = my_joint_traj;
    }

    void set_baxter_arm(std::string& baxter_arm){
        params.baxter_arm = baxter_arm;
    }

    void set_joint_state(sensor_msgs::JointState& jo_state){
        params.my_joint_state = jo_state;
    }

    void set_baxter_arm_joint_values(std::vector<double>& joint_values, std::string& arm){
        if(strcmp(arm.c_str(), "right_arm") == 0)
            params.right_arm_joints = joint_values;
        else
            params.left_arm_joints = joint_values;
    }

    void set_dt(double& dt){
        params.dt = dt;
    }

    void set_baxter_robot_model(robot_model::RobotModelPtr& robot_model_baxter){
        params.robot_model = robot_model_baxter;
    }
};

#endif
