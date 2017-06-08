#include "baxter_throwing_ball_exp/tools.hpp"

// The parameters structure is used by all call backs, main and service
Data_config parameters;
std::ofstream output_file;

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
    ros::Publisher image_publisher = n.advertise<sensor_msgs::Image>("/robot/xdisplay", 1);
    ros::Publisher start_recording_publisher = n.advertise<std_msgs::Bool>("/record_ball_trajectory", true);
    ros::Publisher trajectory_index_publisher = n.advertise<std_msgs::Int64>("/trajectory_index", true);

    ros::AsyncSpinner my_spinner(1);
    my_spinner.start();

    std::string input_file_path, feedback_file_path, baxter_arm, dream_logo;
    std_msgs::Bool ball_trajectory_record;
    double dt;
    bool record = false, execute = false;
    n.getParam("input_file_path", input_file_path);
    n.getParam("feedback_file_path", feedback_file_path);
    n.getParam("baxter_arm", baxter_arm);
    n.getParam("dt", dt);
    n.getParam("rate", parameters.get_rate());
    n.getParam("record", record);
    n.getParam("grap_simulation", parameters.get_grap_ball_simulation());
    n.getParam("epsilon", parameters.get_epsilon());
    n.getParam("execute", execute);
    n.getParam("velocity_option", parameters.get_velocity_option());
    n.getParam("acceleration_option", parameters.get_acceleration_option());
    n.getParam("simulation", parameters.get_simulation());
    n.getParam("check_collision", parameters.get_check_collision());
    n.getParam("start_trajectory_number", parameters.get_start_trajectory_number());
    n.getParam("last_trajectory_number", parameters.get_last_trajectory_number());
    n.getParam("gripper_id", parameters.get_gripper_id("right_gripper"));
    n.getParam("dream_logo_path", dream_logo);


    if(parameters.get_simulation()){
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

    //dispaly_image(dream_logo, image_publisher);

    //test writing

    if(record){
        std::ofstream output_file;
        output_file.open("right_joint_state.csv", std::ofstream::out);
        std::cin.ignore();
        ros::Rate rate(parameters.get_rate());
        while(ros::ok()){
            extract_arm_joints_values(parameters);
            record_arm_joint_trajectory(output_file, parameters);
            rate.sleep();
        }
        output_file.close();
    }
    else{
        for(int i = parameters.get_start_trajectory_number(); i <= parameters.get_last_trajectory_number(); i++){

            //test reading and executing a trajectory
            std::ifstream input_file(input_file_path + std::to_string(i) + ".txt", std::ios_base::in);

            output_file.open(feedback_file_path, std::ofstream::out);

            construct_joint_trajectory_from_file(input_file, parameters);


            if(parameters.get_check_collision()){
                if(is_trajectory_valid(parameters)){
                    if(execute){
                        while(!go_to_initial_position(parameters, ac, gripper_pub))
                            ROS_WARN_STREAM("trying to move to initial position, the action server gave: "
                                     << parameters.get_joint_action_result().result.error_code);

                        execute_joint_trajectory(ac, parameters.get_joint_trajectory(), parameters,
                                                 gripper_pub, start_recording_publisher, trajectory_index_publisher, i);
                    }
                }
                else
                    ROS_ERROR("trajectory not valid !!!!!!!!!");
            }
            else if(execute){
                while(!go_to_initial_position(parameters, ac, gripper_pub))
                    ROS_WARN_STREAM("trying to move to initial position, the action server gave: "
                             << parameters.get_joint_action_result().result.error_code);

                execute_joint_trajectory(ac, parameters.get_joint_trajectory(), parameters,
                                         gripper_pub, start_recording_publisher, trajectory_index_publisher, i);
            }

            input_file.close();
            output_file.close();
            if(parameters.get_grap_ball_simulation())
                delete_model("ball", parameters);
            parameters.set_start_record_feedback(false);
            parameters.set_record(false);
            ROS_INFO_STREAM("trajectory size is: " << parameters.get_joint_trajectory().points.size());
            ROS_WARN_STREAM("finished trajectory: " << i << " press enter for next trajectory");
            ros::Duration my_duration(0);
            parameters.get_action_server_feedback().feedback.actual.time_from_start = my_duration;
            ball_trajectory_record.data = false;
            if(i == 5)
                usleep(1e6);
            start_recording_publisher.publish(ball_trajectory_record);
            std::cin.ignore();
        }
    }

    return 0;
}
