#include "baxter_throwing_ball_exp/tools.hpp"

// The parameters structure is used by all call backs, main and service
Data_config parameters;
std::ofstream output_file;

void joint_state_callback(sensor_msgs::JointState jo_state){
    parameters.set_joint_state(jo_state);
}

void feedback_callback(control_msgs::FollowJointTrajectoryActionFeedback feedback){
    parameters.set_joint_action_feedback(feedback);
    record_feedback(parameters, feedback, output_file);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "throw_ball_naive");
    ros::NodeHandle n;

    //subscribers
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/robot/limb/right/follow_joint_trajectory", true);
    ros::Subscriber right_joint_sub = n.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1, joint_state_callback);
    ros::Subscriber feedback_sub = n.subscribe<control_msgs::FollowJointTrajectoryActionFeedback>("/robot/limb/right/follow_joint_trajectory/feedback", 1, feedback_callback);
    ros::Publisher pub_msg_right = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command",1);

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
    n.getParam("record", record);
    n.getParam("epsilon", parameters.get_epsilon());
    n.getParam("execute", execute);

    parameters.set_baxter_arm(baxter_arm);
    parameters.set_dt(dt);
    parameters.create_model();

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
        //test reading and executing a trajectory
        std::ifstream input_file(input_file_path, std::ios_base::in);

        output_file.open(feedback_file_path, std::ofstream::out);

        construct_joint_trajectory_from_file(input_file, parameters);


        if(is_trajectory_valid(parameters)){
            if(execute){
                go_to_initial_position(parameters, ac);
                execute_joint_trajectory(ac, parameters.get_joint_trajectory(), parameters);
            }
        }
        else
            ROS_ERROR("trajectory not valid !!!!!!!!!");
        input_file.close();
        output_file.close();
        ROS_INFO_STREAM("trajectory size is: " << parameters.get_joint_trajectory().points.size());
    }
    return 0;
}
