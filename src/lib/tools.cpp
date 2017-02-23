#include "../../include/baxter_throwing_ball_exp/tools.hpp"

using namespace std;

typedef vector <double> record_t;
typedef vector <record_t> data_t;


//The function that will execute the trajectory
bool execute_joint_trajectory(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac,
                              trajectory_msgs::JointTrajectory& joint_trajectory){
    if (!ac.waitForServer(ros::Duration(2.0)))
    {
        ROS_ERROR("Could not connect to action server");
        return false;
    }

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = joint_trajectory;
    goal.goal_time_tolerance = ros::Duration(1.0);
    ac.sendGoal(goal);
    if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start + ros::Duration(5)))
    {
        ROS_INFO("Action server reported successful execution");
        return true;
    } else {
        ROS_WARN("Action server could not execute trajectory");
        return false;
    }
}

//check all trajectory points for selfcollision
bool is_trajectory_valid(Data_config& parameters){
    planning_scene::PlanningScene planning_scene(parameters.get_baxter_robot_model());
    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.max_contacts = 100;
    collision_detection::CollisionResult collision_result;
    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    std::filebuf fb;
    fb.open ("test.txt", std::ios::out);
    std::ostream acm_matrix(&fb);

    acm.print(acm_matrix);
    robot_state::RobotState robot_state_holder(parameters.get_baxter_robot_model());

    robot_state_holder.setVariableValues(parameters.get_joint_state());
    for(unsigned i = 0; i < parameters.get_joint_trajectory().points.size(); i++){
        collision_result.clear();
        robot_state_holder.setVariablePositions(parameters.get_baxter_arm_joints_names(parameters.get_baxter_arm()),
                                                parameters.get_joint_trajectory().points[i].positions);
        planning_scene.setCurrentState(robot_state_holder);


        planning_scene.checkSelfCollision(req, collision_result, planning_scene.getCurrentState(), planning_scene.getAllowedCollisionMatrix());
        ROS_INFO_STREAM("Test 6: Current state is "
                        << (collision_result.collision ? "in" : "not in")
                        << " self collision, for iteration: " << i);
        collision_detection::CollisionResult::ContactMap::const_iterator it;
        for(it = collision_result.contacts.begin();
            it != collision_result.contacts.end();
            ++it)
        {
            ROS_INFO("Contact between: %s and %s",
                     it->first.first.c_str(),
                     it->first.second.c_str());
        }

        /*robot_state::RobotState check_state = planning_scene.getCurrentState();
        const double* my_variables = check_state.getVariablePositions();
        for(size_t j = 0; j < check_state.getVariableCount(); j++){
            ROS_WARN_STREAM("joint number: " << j << "with the name: " << check_state.getVariableNames()[j] << " position is: " << my_variables[j]);
        }*/
        if(collision_result.collision)
            return false;
    }
    return true;
}

void extract_arm_joints_values(Data_config& parameters){
    std::vector<std::string> joint_names;
    std::vector<double> joint_values;
    //std::vector<int> joint_index;
    joint_names = parameters.get_baxter_arm_joints_names(parameters.get_baxter_arm());

    for(unsigned i = 0; i < joint_names.size(); ++i){
        joint_values.push_back(parameters.get_joint_state().position[distance(parameters.get_joint_state().name.begin(),
                                                                              find(parameters.get_joint_state().name.begin(),
                                                                                   parameters.get_joint_state().name.end(),
                                                                                   joint_names[i]))]);
    }
    parameters.set_baxter_arm_joint_values(joint_values, parameters.get_baxter_arm());
}

//record arm joint positions into a file
void record_arm_joint_trajectory(ofstream& output_file, Data_config& parameters){
    for(unsigned i = 0; i < parameters.get_baxter_arm_joint_values(parameters.get_baxter_arm()).size(); ++i)
        output_file << parameters.get_baxter_arm_joint_values(parameters.get_baxter_arm())[i] << ",";
    output_file << "\n";
}

//-----------------------------------------------------------------------------
// Let's overload the stream input operator to read a list of CSV fields (which a CSV record).
// Remember, a record is a list of doubles separated by commas ','.
istream& operator >> ( istream& ins, record_t& record )
{
    // make sure that the returned record contains only the stuff we read now
    record.clear();

    // read the entire line into a string (a CSV record is terminated by a newline)
    string line;
    getline( ins, line );

    // now we'll use a stringstream to separate the fields out of the line
    stringstream ss( line );
    string field;
    while (getline( ss, field, ',' ))
    {
        // for each field we wish to convert it to a double
        // (since we require that the CSV contains nothing but floating-point values)
        stringstream fs( field );
        double f = 0.0;  // (default value is 0.0)
        fs >> f;

        // add the newly-converted field to the end of the record
        record.push_back( f );
    }

    // Now we have read a single line, converted into a list of fields, converted the fields
    // from strings to doubles, and stored the results in the argument record, so
    // we just return the argument stream as required for this kind of input overload function.
    return ins;
}

//-----------------------------------------------------------------------------
// Let's likewise overload the stream input operator to read a list of CSV records.
// This time it is a little easier, just because we only need to worry about reading
// records, and not fields.
istream& operator >> ( istream& ins, data_t& data )
{
    // make sure that the returned data only contains the CSV data we read here
    data.clear();

    // For every record we can read from the file, append it to our resulting data
    record_t record;
    while (ins >> record)
    {
        data.push_back( record );
    }

    // Again, return the argument stream as required for this kind of input stream overload.
    return ins;
}

//get largest difference between elements of two vectors
double largest_difference(std::vector<double> &first, std::vector<double> &second){
    Eigen::VectorXd difference(first.size());
    double my_max = 0;
    for(size_t j = 0; j < first.size(); ++j)
        difference(j) = fabs(first[j] - second[j]);
    for(size_t j = 0; j < first.size(); ++j){
        if(difference(j) > my_max)
            my_max = difference(j);
    }
    return my_max;
}

//git rid of doubled vectors in a a vector of vectors
void optimize_vector_of_vectors(std::vector<std::vector<double>>& vector_to_optimize){
    //make a copy to work with and another to be the output
    std::vector<std::vector<double>> working_copy = vector_to_optimize, output_copy;
    for(unsigned i = 0; i < working_copy.size(); i++)
        for(unsigned j = 1; j < working_copy.size(); j++)
            if(largest_difference(working_copy[i], working_copy[j]) < 0.0001)
                working_copy.erase(working_copy.begin() + j);
    vector_to_optimize = working_copy;
}

void construct_joint_trajectory_from_vector(trajectory_msgs::JointTrajectory& my_joint_trajectory, data_t& raw_joint_traj, double& dt){
    double t_ = 0.0;
    for(unsigned i = 0; i < raw_joint_traj.size(); i++){
        trajectory_msgs::JointTrajectoryPoint pt;
        pt.positions = raw_joint_traj[i];
        pt.velocities.resize(raw_joint_traj[i].size(), 0.0);
        pt.accelerations.resize(raw_joint_traj[i].size(), 0.0);
        pt.effort.resize(raw_joint_traj[i].size(), 0.0);
        pt.time_from_start = ros::Duration(i*dt);
        t_+=i*dt;
        my_joint_trajectory.points.push_back(pt);
    }
}

//The function that will construct the joint trajectory
void construct_joint_trajectory_from_file(std::ifstream& text_file, Data_config& parameters){
    //Extract all joints waypoints and delta_t from textfile
    // Here is the data we want.
    data_t data;
    text_file >> data;
    // Complain if something went wrong.
    if (!text_file.eof())
    {
        ROS_ERROR("Fooey!");
        return ;
    }
    text_file.close();
    //delete doupled values
    optimize_vector_of_vectors(data);

    //for printing all joints values
    /*for(unsigned i = 0; i < data.size(); i++){
        for(unsigned j = 0; j < data[i].size(); j++)
            ROS_INFO_STREAM("this element is: " << data[i][j]);
        ROS_INFO("*****************************************");
    }*/

    //check that all waypoints

    trajectory_msgs::JointTrajectory my_joint_trajectory;
    my_joint_trajectory.joint_names = parameters.get_baxter_arm_joints_names(parameters.get_baxter_arm());
    construct_joint_trajectory_from_vector(my_joint_trajectory, data, parameters.get_dt());

    //set the trajectory in the parameters
    parameters.set_joint_trajectory(my_joint_trajectory);
}
