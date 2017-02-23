#include "parameters.hpp"
#include "baxter_throwing_ball_exp/tools.hpp"

// The parameters structure is used by all call backs, main and service
Data_config parameters;

void joint_state_callback(sensor_msgs::JointState jo_state){
    parameters.set_joint_state(jo_state);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "throw_ball_naive");
    ros::NodeHandle n;

    //subscribers
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/robot/limb/right/follow_joint_trajectory", true);
    ros::Subscriber right_joint_sub = n.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1, joint_state_callback);

    ros::AsyncSpinner my_spinner(1);
    my_spinner.start();

    std::string file_path, baxter_arm;
    double dt;
    bool record = false;
    n.getParam("file_path", file_path);
    n.getParam("baxter_arm", baxter_arm);
    n.getParam("dt", dt);
    n.getParam("record", record);

    parameters.set_baxter_arm(baxter_arm);
    parameters.set_dt(dt);
    parameters.create_model();

    //test writing
    if(record){
        std::ofstream output_file;
        output_file.open("right_joint_state.csv", std::ofstream::out);
        std::cin.ignore();
        ros::Rate rate(10);
        while(ros::ok()){
            extract_arm_joints_values(parameters);
            record_arm_joint_trajectory(output_file, parameters);
            rate.sleep();
        }
        output_file.close();
    }
    else{
        //test reading and executing a trajectory
        std::ifstream input_file(file_path, std::ios_base::in);

        construct_joint_trajectory_from_file(input_file, parameters);

        ROS_INFO_STREAM("trajectory size is: " << parameters.get_joint_trajectory().points.size());
        if(is_trajectory_valid(parameters))
            execute_joint_trajectory(ac, parameters.get_joint_trajectory());
        else
            ROS_ERROR("trajectory not valid !!!!!!!!!");
    }
    return 0;
}
