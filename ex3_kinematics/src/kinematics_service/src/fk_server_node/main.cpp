#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "kinematics_service_msgs/ComputeFK.h"
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>

bool compute_fk(kinematics_service_msgs::ComputeFK::Request &request, kinematics_service_msgs::ComputeFK::Response &response){
    
    //Using the RobotModel, we can construct a RobotState that maintains the configuration of the robot
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    const bool success = moveit::core::robotStateMsgToRobotState(request.robot_state,*kinematic_state);
    if(!success){
        ROS_INFO("Error in robotStateMsgToRogetGlobalLinkTransformotState() function");
        return false;
    }

    // Get the planning group name from the parameter server
    ros::NodeHandle nh;
    /*std::string planning_group_name;
    
    if(!nh.getParam("planning_group_name", planning_group_name))
    {
        ROS_ERROR("'planning_group_name' is undefined on the parameter server");
        return false;
    }*/

    const std::string modelGroup = kinematic_model->getSRDF()->getGroups()[0].name_;

    // joint_model_group represents the robot model for planning_group_name
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(modelGroup);
    
    // Take link name
    const std::vector<std::string>& link_names = joint_model_group->getLinkModelNames();

    //ComputeFk and prepare the response message
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform(link_names.back());

    tf::poseEigenToMsg(end_effector_state, response.end_effector_pose);

    return true;

}

int main(int argc, char** argv){

    // Init node
    ros::init(argc,argv,"FK_server");
    ros::NodeHandle nh;

    //Start service

    ros::ServiceServer service=nh.advertiseService("compute_forward_kinematics",compute_fk);

    ros::spin();
    return 0;

}