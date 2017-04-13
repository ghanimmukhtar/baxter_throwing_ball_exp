#include "baxter_throwing_ball_exp/tools.hpp"

// The parameters structure is used by all call backs, main and service
Data_config parameters;
std::ofstream output_file;

void trajectory_index_cb(const std_msgs::Int64::ConstPtr& index_msgs){
    parameters.set_selected_trajectory_index(index_msgs->data);
}

void joint_state_callback(sensor_msgs::JointState jo_state){
    parameters.set_joint_state(jo_state);
}

void feedback_callback(control_msgs::FollowJointTrajectoryActionFeedback feedback){
    //parameters.set_joint_action_feedback(feedback);
    if(parameters.get_start_record_feedback()){
        parameters.set_action_server_feedback(feedback);
        record_feedback(parameters, feedback, output_file);
    }
}

void status_callback(actionlib_msgs::GoalStatusArray status){
    parameters.set_action_server_status(status);
}

void result_callback(control_msgs::FollowJointTrajectoryActionResult result){
    parameters.set_action_server_result(result);
}

//call back that register end effector pose and rearrange the orientation in RPY
void right_eef_Callback(baxter_core_msgs::EndpointState r_eef_feedback){
    locate_eef_pose(r_eef_feedback.pose, parameters, "right_gripper");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "throw_ball_naive");
    ros::NodeHandle n;

    //subscribers
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/robot/limb/right/follow_joint_trajectory", true);
    ros::Subscriber sub_r_eef_msg = n.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state", 10, right_eef_Callback);
    ros::Subscriber right_joint_sub = n.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1, joint_state_callback);
    ros::Subscriber feedback_sub = n.subscribe<control_msgs::FollowJointTrajectoryActionFeedback>("/robot/limb/right/follow_joint_trajectory/feedback", 1, feedback_callback);
    //ros::Subscriber status_sub = n.subscribe<actionlib_msgs::GoalStatusArray>("/robot/limb/right/follow_joint_trajectory/status", 1, status_callback);
    ros::Subscriber result_sub = n.subscribe<control_msgs::FollowJointTrajectoryActionResult>("/robot/limb/right/follow_joint_trajectory/result", 1, result_callback);
    ros::Publisher gripper_pub = n.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", true);
    ros::Subscriber choosen_trajectory_sub = n.subscribe<std_msgs::Int64>("/trajselector/trajectory_index", 1, trajectory_index_cb);

    ros::AsyncSpinner my_spinner(1);
    my_spinner.start();

    std::string input_file_path, feedback_file_path, baxter_arm;
    double dt;
    bool record = false, execute = false;
    n.getParam("input_file_path", input_file_path);
    n.getParam("feedback_file_path", feedback_file_path);
    n.getParam("baxter_arm", baxter_arm);
    n.getParam("dt", dt);
    n.getParam("rate", parameters.get_rate());
    n.getParam("release_ball_dt", parameters.get_release_ball_dt());
    n.getParam("record", record);
    n.getParam("grap_simulation", parameters.get_grap_ball_simulation());
    n.getParam("epsilon", parameters.get_epsilon());
    n.getParam("execute", execute);
    n.getParam("velocity_option", parameters.get_velocity_option());
    n.getParam("acceleration_option", parameters.get_acceleration_option());
    n.getParam("simulation", parameters.get_simulation());
    n.getParam("check_collision", parameters.get_check_collision());
    n.getParam("gripper_id", parameters.get_gripper_id("right_gripper"));

    if(parameters.get_grap_ball_simulation()){
        ros::ServiceClient spawner = n.serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");
        ros::ServiceClient state = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        ros::ServiceClient deleter = n.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
        ros::ServiceClient ik = n.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/right/PositionKinematicsNode/IKService");
        parameters.set_gazebo_model_spawner(spawner);
        parameters.set_gazebo_model_state_clt(state);
        parameters.set_gazebo_model_delete_clt(deleter);
        parameters.set_baxter_right_arm_ik(ik);
    }



    parameters.set_baxter_arm(baxter_arm);
    parameters.set_dt(dt);
    parameters.create_model();



    //test reading and executing a trajectory
    while(parameters.get_selected_trajectory_index() == -1);

    std::ifstream input_file(input_file_path + std::to_string(parameters.get_selected_trajectory_index()) + ".txt", std::ios_base::in);

    output_file.open(feedback_file_path, std::ofstream::out);

    construct_joint_trajectory_from_file(input_file, parameters);


    if(parameters.get_check_collision()){
        if(is_trajectory_valid(parameters)){
            if(execute){
                while(!go_to_initial_position(parameters, ac, gripper_pub))
                    ROS_WARN_STREAM("trying to move to initial position, the action server gave: "
                                    << parameters.get_joint_action_result().result.error_code);
                execute_joint_trajectory(ac, parameters.get_joint_trajectory(), parameters, gripper_pub);
            }
        }
        else
            ROS_ERROR("trajectory not valid !!!!!!!!!");
    }
    else if(execute){
        while(!go_to_initial_position(parameters, ac, gripper_pub))
            ROS_WARN_STREAM("trying to move to initial position, the action server gave: "
                            << parameters.get_joint_action_result().result.error_code);
        execute_joint_trajectory(ac, parameters.get_joint_trajectory(), parameters, gripper_pub);
    }

    input_file.close();
    output_file.close();


    parameters.set_start_record_feedback(false);
    parameters.set_record(false);
    ROS_INFO_STREAM("trajectory size is: " << parameters.get_joint_trajectory().points.size());
    ros::Duration my_duration(0);
    parameters.get_action_server_feedback().feedback.actual.time_from_start = my_duration;
    std::cin.ignore();
    if(parameters.get_grap_ball_simulation())
        delete_model("ball", parameters);


    return 0;
}
