#ifndef __SIM_PARAM_HPP__
#define __SIM_PARAM_HPP__
#include <ros/ros.h>
#include <string>
#include <math.h>

#include <Eigen/Core>
#include <vector>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <control_msgs/FollowJointTrajectoryActionFeedback.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SpawnModel.h>
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
    control_msgs::FollowJointTrajectoryActionFeedback right_joint_action_feedback;
    double dt, epsilon, rate, start_time;
    robot_model::RobotModelPtr robot_model;
    bool first = false, record = false, velocity_option = false, acceleration_option = false, simulation = true, check_collision = true;
    int point_count, start_trajectory_number = 1, last_trajectory_number = 1;

    ros::ServiceClient gazebo_spawn_clt, gazebo_model_state, gazebo_model_delete, baxter_right_arm_ik_solver;

    std::string package_name = "baxter_throwing_ball_exp";
    geometry_msgs::Pose table_pose, ball_pose;

    geometry_msgs::Pose l_eef_pose, r_eef_pose;
    Eigen::VectorXd l_eef_rpy_pose, r_eef_rpy_pose;
    Eigen::Vector3d l_eef_position, r_eef_position;
    Eigen::Vector3d l_eef_rpy_orientation, r_eef_rpy_orientation;

    int right_gripper_id, left_gripper_id;
    bool grap_ball_simulation = false;
};

class Data_config{

public:
    Parameters params;
    /*Data_config(){
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        params.robot_model = robot_model_loader.getModel();
    }*/

    void configure_object_pose(){
        //table
        params.table_pose.position.x = 0.7;
        params.table_pose.position.y = 0.0;
        params.table_pose.position.z = -0.93;
        tf::Quaternion tmp_orientation;
        tmp_orientation.setRPY(0.0,
                               0.0,
                               M_PI/2.0);
        params.table_pose.orientation.w = tmp_orientation.getW();
        params.table_pose.orientation.x = tmp_orientation.getX();
        params.table_pose.orientation.y = tmp_orientation.getY();
        params.table_pose.orientation.z = tmp_orientation.getZ();
        //ball
        params.ball_pose.position.x = 0.5;
        params.ball_pose.position.y = -0.1;
        params.ball_pose.position.z = -0.1;
        params.ball_pose.orientation.w = 1.0;
    }

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

    control_msgs::FollowJointTrajectoryActionFeedback& get_joint_action_feedback(){
            return params.right_joint_action_feedback;
    }

    double& get_dt(){
        return params.dt;
    }

    double& get_rate(){
        return params.rate;
    }

    double& get_epsilon(){
        return params.epsilon;
    }

    double& get_start_time(){
        return params.start_time;
    }

    robot_model::RobotModelPtr& get_baxter_robot_model(){
        return params.robot_model;
    }

    bool get_first(){
        return params.first;
    }

    bool get_record(){
        return params.record;
    }

    bool& get_velocity_option(){
        return params.velocity_option;
    }

    bool& get_acceleration_option(){
        return params.acceleration_option;
    }

    bool& get_simulation(){
        return params.simulation;
    }

    bool& get_check_collision(){
        return params.check_collision;
    }

    int& get_point_count(){
        return params.point_count;
    }

    int& get_start_trajectory_number(){
        return params.start_trajectory_number;
    }

    int& get_last_trajectory_number(){
        return params.last_trajectory_number;
    }

    ros::ServiceClient& get_gazebo_model_spawner(){
        return params.gazebo_spawn_clt;
    }

    ros::ServiceClient& get_gazebo_model_state_client(){
        return params.gazebo_model_state;
    }

    ros::ServiceClient& get_gazebo_model_delete_client(){
        return params.gazebo_model_delete;
    }

    ros::ServiceClient& get_baxter_right_arm_ik(){
        return params.baxter_right_arm_ik_solver;
    }

    std::string& get_package_name(){
        return params.package_name;
    }

    geometry_msgs::Pose& get_object_pose(std::string object){
        if(strcmp(object.c_str(), "table") == 0)
            return params.table_pose;
        else if(strcmp(object.c_str(), "ball") == 0)
            return params.ball_pose;

        //else
          //  ROS_WARN("please provide string argument with value of: 'table' or 'ball'");
        //return NULL;
    }

    geometry_msgs::Pose& get_eef_pose(const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            return params.l_eef_pose;
        else
            return params.r_eef_pose;
    }

    Eigen::VectorXd& get_eef_rpy_pose(const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            return params.l_eef_rpy_pose;
        else
            return params.r_eef_rpy_pose;
    }

    Eigen::Vector3d& get_eef_position(const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            return params.l_eef_position;
        else
            return params.r_eef_position;
    }

    Eigen::Vector3d& get_eef_rpy_orientation(const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            return params.l_eef_rpy_orientation;
        else
            return params.r_eef_rpy_orientation;
    }

    int& get_gripper_id(const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            return params.left_gripper_id;
        else
            return params.right_gripper_id;
    }

    bool get_grap_ball_simulation(){
        return params.grap_ball_simulation;
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

    void set_rate(double& rate){
        params.rate = rate;
    }

    void set_start_time(double start){
        params.start_time = start;
    }

    void set_first(bool first){
        params.first = first;
    }

    void set_record(bool record){
        params.record = record;
    }

    void set_velocity_option(bool velocity_option){
        params.velocity_option = velocity_option;
    }

    void set_acceleration_option(bool acceleration_option){
        params.acceleration_option = acceleration_option;
    }

    void set_simulation(bool simulation){
        params.simulation = simulation;
    }

    void set_check_collision(bool collision){
        params.check_collision = collision;
    }

    void set_baxter_robot_model(robot_model::RobotModelPtr& robot_model_baxter){
        params.robot_model = robot_model_baxter;
    }

    void set_joint_action_feedback(control_msgs::FollowJointTrajectoryActionFeedback& feedback){
            params.right_joint_action_feedback = feedback;
    }

    void set_epsilon(double& epsilon){
        params.epsilon = epsilon;
    }

    void set_point_count(int count){
        params.point_count = count;
    }

    void set_start_trajectory_number(int traj_number){
        params.start_trajectory_number = traj_number;
    }

    void set_last_trajectory_number(int traj_number){
        params.last_trajectory_number = traj_number;
    }

    void set_gazebo_model_spawner(ros::ServiceClient& spawner){
        params.gazebo_spawn_clt = spawner;
    }

    void set_gazebo_model_state_clt(ros::ServiceClient& state){
        params.gazebo_model_state = state;
    }

    void set_gazebo_model_delete_clt(ros::ServiceClient& deleter){
        params.gazebo_model_delete = deleter;
    }

    void set_baxter_right_arm_ik(ros::ServiceClient& solver){
        params.baxter_right_arm_ik_solver = solver;
    }

    void set_package_name(std::string& pkg_name){
        params.package_name = pkg_name;
    }
    void set_eef_rpy_pose(Eigen::VectorXd& eef_rpy_pose, const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            params.l_eef_rpy_pose = eef_rpy_pose;
        else
            params.r_eef_rpy_pose = eef_rpy_pose;
    }

    void set_eef_position(Eigen::Vector3d& eef_position, const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            params.l_eef_position = eef_position;
        else
            params.r_eef_position = eef_position;
    }

    void set_eef_rpy_orientation(Eigen::Vector3d& eef_rpy_orientation, const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            params.l_eef_rpy_orientation = eef_rpy_orientation;
        else
            params.r_eef_rpy_orientation = eef_rpy_orientation;
    }

    void set_eef_pose(geometry_msgs::Pose& eef_pose, const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            params.l_eef_pose = eef_pose;
        else
            params.r_eef_pose = eef_pose;
    }

    void set_gripper_id(int id, const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            params.left_gripper_id = id;
        else
            params.right_gripper_id = id;
    }
};

#endif
