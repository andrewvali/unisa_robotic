#include <actionlib/server/simple_action_server.h>
#include <kinematics_action/inverse_kinematics_action.h>
#include <moveit/robot_state/conversions.h>
#include <angles/angles.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit_msgs/MoveItErrorCodes.h>

// Initialization
kinematics_action::InverseKinematicsAction::InverseKinematicsAction():
    action_server_(nh_,"ik_solver", boost::bind(&kinematics_action::InverseKinematicsAction::ik_callback_, this, _1), false),
    robot_model_loader_(new robot_model_loader::RobotModelLoader("robot_description")), 
    kinematic_model_(robot_model_loader_->getModel()),
    jmg_(nullptr)
{
    std::string planning_group_name;
    if(!nh_.getParam("planning_group_name", planning_group_name))
    {
        ROS_ERROR("'planning_group_name' is undefined on the parameter server");
        return;
    }

    jmg_ = kinematic_model_-> getJointModelGroup(planning_group_name);

    // Start the server
    action_server_.start();

}

// Decostructor not implemented 
kinematics_action::InverseKinematicsAction::~InverseKinematicsAction(){

}

void kinematics_action::InverseKinematicsAction::ik_callback_(const kinematics_action_msgs::GetInverseKinematicsSolutionGoalConstPtr & goal)
{
    ROS_INFO("IK solver: goal received");
    // get the instance of kinematic solver, in this case KDL 
    const kinematics::KinematicsBaseConstPtr solver = jmg_->getSolverInstance();
    kinematics_action_msgs::GetInverseKinematicsSolutionResult result;
    int ik_calls_counter = 0;

    while(ik_calls_counter < 2000 && ros::ok()){
        /* setting a random seed for starting randomly the new solutions research for maximizing the possibility of finding new solutions
        solution is an array of solutions for storing the found ones*/ 
        std::vector<double> seed_state = generateSeedState_();
        std::vector<double> solution;
        moveit_msgs::MoveItErrorCodes error_code;

        // compute a IK solution using KDL solver 
        solver->getPositionIK(goal->pose, seed_state, solution, error_code);
        if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
            // check if the found solution is new
            normalizeJointPositions_(solution);
            if(isSolutionNew_(solution)){
                // if the solution is new, it is added in solutions array and it updates the robot state basing on the found solution
                ik_solutions_.push_back(solution);
                // creation of feedback message
                moveit::core::RobotState robot_state(kinematic_model_);
                robot_state.setVariablePositions(solution);
                kinematics_action_msgs::GetInverseKinematicsSolutionFeedback feedback;
                moveit::core::robotStateToRobotStateMsg(robot_state, feedback.ik_solution);
                action_server_.publishFeedback(feedback);
                // adding the feedback to the final response 
                result.ik_solutions.push_back(feedback.ik_solution);
            }
        }
        ik_calls_counter++;
    }
    if(ik_solutions_.size() == 0)
        action_server_.setAborted(result, "could not find any IK solution");
    else {
        std::ostringstream ss;
        ss << "found " << ik_solutions_.size() << " IK Solutons";
        action_server_.setSucceeded(result, ss.str());

    }
    ik_solutions_.resize(0);
}

// check if the current soliution is equal to another previously found
bool kinematics_action::InverseKinematicsAction::isSolutionNew_(const std::vector<double> & solution) const{
    for (int i= 0; i<ik_solutions_.size(); i++){
        bool are_solutions_equal = true;

        for(int j= 0; j< ik_solutions_[i].size() && are_solutions_equal; j++){
            double diff;
            if(jmg_->getActiveJointModels()[j]-> getType() == robot_model::JointModel::REVOLUTE)
                diff = angles::shortest_angular_distance(ik_solutions_[i][j], solution[j]);
            else 
                diff = ik_solutions_[i][j] - solution[j];
            // tollerance on the solution
            if(std::fabs(diff)>1e-3)
                are_solutions_equal = false;
        }
        if(are_solutions_equal)
            return false;
    }

    return true;
}
// seeds are generated within the joint boundes
std::vector<double> kinematics_action::InverseKinematicsAction::generateSeedState_() const{
    std::vector<double> seed_state;
    std::vector<std::string> joint_names = kinematic_model_->getVariableNames();
    for(int i = 0; i<joint_names.size(); i++){
        double lb = kinematic_model_->getURDF()->getJoint(joint_names[i])->limits->lower;
        double ub = kinematic_model_->getURDF()->getJoint(joint_names[i])->limits->upper;
        double span = ub-lb;
        seed_state.push_back((double)std::rand()/RAND_MAX*span + lb);
    }
    return seed_state;
}

// return a vector of angles beetwen (-pi, pi) because it useful to compare joint angles with different boundes 
void kinematics_action::InverseKinematicsAction::normalizeJointPositions_(std::vector<double> & solution) const{
    for(int i=0;i<solution.size();i++){
        if(jmg_->getActiveJointModels()[i]->getType() == robot_model::JointModel::REVOLUTE)
            solution[i] = angles::normalize_angle(solution[i]);
    }
}







