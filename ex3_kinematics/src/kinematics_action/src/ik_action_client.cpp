#include <kinematics_action/inverse_kinematics_action.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>
#include <condition_variable>
#include <actionlib/client/simple_action_client.h>

void handleGoalCompletionEvent(const actionlib::SimpleClientGoalState & state, const kinematics_action_msgs::GetInverseKinematicsSolutionResultConstPtr & result);
void serializeIKSolution(std::ostringstream & ss, const moveit_msgs::RobotState & robot_state);
void handleGoalActiveEvent();
void handleFeedbackEvent(const kinematics_action_msgs::GetInverseKinematicsSolutionFeedbackConstPtr & feedback);

std::mutex mutex;
std::condition_variable result_handled;

int main(int argc,char **argv)
{
    ros::init(argc, argv, "kinematics_action_client");
    // action client creation
    actionlib::SimpleActionClient<kinematics_action_msgs::GetInverseKinematicsSolutionAction> client("ik_solver", true);
    // waiting the server instantiation
    client.waitForServer();
    // goal creation
    kinematics_action_msgs::GetInverseKinematicsSolutionGoal goal;

    goal.pose.position.x = 1.0;
    goal.pose.position.y = 0.0;
    goal.pose.position.z = 1.0;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, 0.0);

    goal.pose.orientation = tf2::toMsg(quaternion);

    // send the goal to the action server and set the corresponding callbacks
    client.sendGoal(goal, &handleGoalCompletionEvent, &handleGoalActiveEvent, &handleFeedbackEvent);

    // waiting the server response 
    if(!client.waitForResult(ros::Duration(30.0)))
        ROS_ERROR("The IK solver did not complete in the allotted time");

    /* the client can terminate the execution only when it has been completed the HandleGoalCompletionEvent callback
    */
    std::unique_lock<std::mutex> lock(mutex);
    result_handled.wait(lock);

    ros::shutdown();
    return 0;
}

void handleGoalCompletionEvent(const actionlib::SimpleClientGoalState & state, const kinematics_action_msgs::GetInverseKinematicsSolutionResultConstPtr & result)
{
    std::ostringstream ss;
    // check the goal state: if at least one solution was found for the current goal, this state was setted on SUCCEEDED
    if(state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
    {
        int num_of_solutions = result -> ik_solutions.size();

        ss << "Goal achieved. " << state.getText() << std::endl;

        // for all solutions, print joints position
        for(int i=0; i < num_of_solutions; i++)
        {
            serializeIKSolution(ss, result -> ik_solutions[i]);
            ss << std::endl;
        }

        ROS_INFO_STREAM(ss.str());

        ros::NodeHandle nh;

        //Instantiate the joint state publisher publishing on the joint_state topic
        ros::Publisher joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states",1);

        //load the robot model
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model::RobotModelConstPtr kinematic_model = robot_model_loader.getModel();

        //get the planning group name from the parameter server
        std::string planning_group_name;

        if(!nh.getParam("planning_group_name", planning_group_name))
        {
            ROS_ERROR("'planning group name' is undefined on the parameter server");
        }

        //get the planning group
        const robot_state::JointModelGroup *joint_model_group = kinematic_model -> getJointModelGroup(planning_group_name);

        sensor_msgs::JointState joint_state_msg;

        joint_state_msg.name = joint_model_group -> getVariableNames();
        ROS_INFO("Publishing solutions...");

        ros::Duration sleep_time(2.0);
        
        // publish the found solutions on joint_states topic for moving the robot and see the solutions every 2 seconds
        for(int i=0; i < num_of_solutions; i++)
        {
            sleep_time.sleep();

            joint_state_msg.position = result -> ik_solutions[i].joint_state.position;
            joint_state_msg.header.stamp = ros::Time::now();

            joint_state_publisher.publish(joint_state_msg);
        }

        ROS_INFO("All solutions published!");

    }
    else
    {
        ss << "Goal aborted. " << state.getText();
        ROS_INFO_STREAM(ss.str());
    }

    result_handled.notify_all();
}

void handleGoalActiveEvent(){
    ROS_INFO("Inverse kinematics request sent to the IK resolution action server");
}

// print the single new solution found 
void handleFeedbackEvent(const kinematics_action_msgs::GetInverseKinematicsSolutionFeedbackConstPtr & feedback)
{
    std::ostringstream ss;

    ss << "Received IK solution: ";

    serializeIKSolution(ss, feedback -> ik_solution);

    ROS_INFO_STREAM(ss.str());
}

// formatting the single new solution found in custom String 
void serializeIKSolution(std::ostringstream & ss, const moveit_msgs::RobotState & robot_state)
{
    int n_joints = robot_state.joint_state.position.size();
    ss << "[";

    for(int i=0; i<n_joints; i++){
    ss << robot_state.joint_state.position[i];
        if(i != n_joints - 1) ss << ", ";
    
    }

    ss << "]";
    
}