#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "kinematics_service_msgs/ComputeFK.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/GetPositionFK.h>

int main(int argc, char** argv){
    //Init node
    ros::init(argc,argv,"FK_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<kinematics_service_msgs::ComputeFK>("compute_forward_kinematics");
    ros::ServiceClient test_client = nh.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");

    ros::Rate rate= ros::Rate(0.1);
    rate.sleep();

    //Using the RobotModel, we can construct a RobotState that maintains the configuration of the robot
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));


    // Service request
    kinematics_service_msgs::ComputeFK fk_service;
    // Service request to calculated fk using /compute_fk
    moveit_msgs::GetPositionFK test_service;

    kinematic_state->setToDefaultValues();
    const std::string model_frame = kinematic_model->getModelFrame();

    // Take planning group name
    const std::string modelGroup = kinematic_model->getSRDF()->getGroups()[0].name_;

    // joint_model_group represents the robot model for planning_group_name
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(modelGroup);
    //const std::vector<string> link_names = kinematic_model->getLinkModels();
    std::vector<double> jointValues;

    moveit::core::robotStateToRobotStateMsg(*kinematic_state,fk_service.request.robot_state);
    

    while(nh.ok()){
        if(!client.call(fk_service)){
            ROS_INFO("Problem computing Forward Kinematics"); 
        }

        // Convert orientation in RPY Angles
        tf2::Quaternion quaternion;
        tf2::fromMsg(fk_service.response.end_effector_pose.orientation,quaternion);

        tf2::Matrix3x3 matrix(quaternion);
        tf2Scalar roll,pitch,yaw;
        matrix.getRPY(roll,pitch,yaw);

        // Print Results
        // Print solution to stdout
        std::ostringstream output_msg;

        output_msg << "Computed forward kinematic solution with custom solver:"  << std::endl;
        output_msg << "Position (XYZ): ["; 
        output_msg << fk_service.response.end_effector_pose.position.x << ", ";
        output_msg << fk_service.response.end_effector_pose.position.y << ", ";
        output_msg << fk_service.response.end_effector_pose.position.z << "]" << std::endl;
        output_msg << "Orientation (RPY): [" << roll << ", " << pitch << ", " << yaw << "]\n\n";

        ROS_INFO_STREAM(output_msg.str());
        output_msg.clear();

        // Compare kinematic solution calculated with the service /compute_fk of the move_group node

        //Prepare the request 
        std::vector<std::string> link_names = kinematic_model->getLinkModelNames();

        test_service.request.header.frame_id = link_names[0];
        test_service.request.fk_link_names.push_back(link_names.back());
        moveit::core::robotStateToRobotStateMsg(*kinematic_state,test_service.request.robot_state);

        if(!test_client.call(test_service)){
            ROS_INFO("Problem computing FK with /compute_fk servide of move_group_node");
        }
        tf2::fromMsg(test_service.response.pose_stamped[0].pose.orientation,quaternion);
        tf2::Matrix3x3 rotation_matrix(quaternion);
        matrix.getRPY(roll,pitch,yaw);

        output_msg << "Computed forward kinematic solution with /compute_fk of move_group_node:"  << std::endl;
        output_msg << "Position (XYZ): ["; 
        output_msg << test_service.response.pose_stamped[0].pose.position.x << ", ";
        output_msg << test_service.response.pose_stamped[0].pose.position.y << ", ";
        output_msg << test_service.response.pose_stamped[0].pose.position.z << "]" << std::endl;
        output_msg << "Orientation (RPY): [" << roll << ", " << pitch << ", " << yaw << "]";
        ROS_INFO_STREAM(output_msg.str());
        output_msg.clear();

        rate.sleep();

        ROS_INFO("\n\n********* New Configuration *********\n");
        kinematic_state->setToRandomPositions(joint_model_group);

        moveit::core::robotStateToRobotStateMsg(*kinematic_state, fk_service.request.robot_state);

        



    }

    ros::shutdown();
    return 0;
}