#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <thread>

#include <cable_model/CableMsg.h>

#include "trajectory_msgs/JointTrajectory.h"

#include <iostream>

#include <ur_cable_manipulation/Jacobian.hpp>
#include <Eigen/Dense>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>


// int main(int argc, char **argv)
// {
//     // Initialize the ROS node
//     ros::init(argc, argv, "demo_jacobian");
//     ros::NodeHandle nh;

//     ros::AsyncSpinner spinner(2); 
//     spinner.start();


//     static const std::string ur5e_PLANNING_GROUP = "ur5e_arm";
//     static const std::string ur5e_HAND_PLANNING_GROUP = "ur5e_hand";

//     moveit::planning_interface::MoveGroupInterface ur5e_move_group_interface(ur5e_PLANNING_GROUP);
//     moveit::planning_interface::MoveGroupInterface ur5e_hand_move_group_interface(ur5e_HAND_PLANNING_GROUP);

//     const robot_state::JointModelGroup* ur5e_joint_model_group = ur5e_move_group_interface.getCurrentState()->getJointModelGroup(ur5e_PLANNING_GROUP);
//     const robot_state::JointModelGroup* ur5e_hand_joint_model_group = ur5e_hand_move_group_interface.getCurrentState()->getJointModelGroup(ur5e_HAND_PLANNING_GROUP);

//     ur5e_move_group_interface.allowReplanning(true); 
//     ur5e_move_group_interface.setNumPlanningAttempts(10); 

//     ur5e_hand_move_group_interface.allowReplanning(true); 
//     ur5e_hand_move_group_interface.setNumPlanningAttempts(10);

//     const robot_model::RobotModelConstPtr& robot_model = ur5e_move_group_interface.getRobotModel();
//     robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model));


//     ROS_INFO_NAMED("PLANNING","START PLANNING\n\n\n");

//     moveit::core::RobotStatePtr ur5e_current_state = ur5e_move_group_interface.getCurrentState();
//     std::vector<double> ur5e_joint_group_positions;
//     ur5e_current_state->copyJointGroupPositions(ur5e_joint_model_group, ur5e_joint_group_positions);

//     moveit::core::RobotStatePtr ur5e_hand_current_state = ur5e_hand_move_group_interface.getCurrentState();
//     std::vector<double> ur5e_hand_joint_group_positions;
//     ur5e_hand_current_state->copyJointGroupPositions(ur5e_hand_joint_model_group, ur5e_hand_joint_group_positions);



//     ROS_INFO_NAMED("PLANNING", "Planning frame: %s\n", ur5e_move_group_interface.getPlanningFrame().c_str());
//     ROS_INFO_NAMED("PLANNING", "End effector link: %s\n", ur5e_move_group_interface.getEndEffectorLink().c_str());
//     ROS_INFO_NAMED("PLANNING", "Reference frame: %s\n", ur5e_move_group_interface.getPoseReferenceFrame().c_str());
//     std::cout<<"\n\n\n\n"<<std::endl;

//     // Get the current joint values
//     std::vector<double> joint_values;
//     ur5e_move_group_interface.getCurrentState()->copyJointGroupPositions(ur5e_move_group_interface.getCurrentState()->getRobotModel()->getJointModelGroup(ur5e_move_group_interface.getName()), joint_values);

//     for (std::size_t i = 0; i < 6; ++i)
//     {
//     ROS_INFO("Joint %d: %f", i, joint_values[i]);
//     }
//     // Set the joint values in the kinematic state
//     kinematic_state->setJointGroupPositions(ur5e_move_group_interface.getName(), joint_values);

//     // Get the Jacobian matrix
//     const Eigen::MatrixXd& moveit_jacobian = kinematic_state->getJacobian(kinematic_state->getJointModelGroup(ur5e_move_group_interface.getName()), );

//     std::cout<<"MOVEIT jacobian: "<<moveit_jacobian<<std::endl;

//     std::cout<<"\n\n----------------\n\n"<<std::endl;

//     std::vector<double> current_joint_values;// = ur5e_move_group_interface.getCurrentJointValues();
//     ur5e_move_group_interface.getCurrentState()->copyJointGroupPositions(ur5e_move_group_interface.getCurrentState()->getRobotModel()->getJointModelGroup(ur5e_move_group_interface.getName()), current_joint_values);
//     //compute jacobian
//     Eigen::MatrixXd custom_jacobian = Jacobian::getJacobian_ur5e_wrt_bl(-1.564, -0.762, -1.764, 3.91, 1.567, 0.0);
//     std::cout<<"CUSTOM jacobian: "<<custom_jacobian<<std::endl;

//     // const Eigen::MatrixXd& jacobian = ur5e_move_group_interface.getJacobianMatrix();
//     const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("ur5e_wrist_3_link");

//     /* Print end-effector pose. Remember that this is in the model frame */
//     ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
//     ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
//     ros::shutdown();
//     return 0;
// }

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "demo_jacobian");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2); 
    spinner.start();


    static const std::string ur5e_PLANNING_GROUP = "manipulator";
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(ur5e_PLANNING_GROUP);

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
   kinematic_state->getJacobian(joint_model_group,
                             kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                             reference_point_position, jacobian);

    std::cout << kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back())->getName() << std::endl;
    ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");

    moveit::planning_interface::MoveGroupInterface ur5e_mgi(ur5e_PLANNING_GROUP);
    std::vector<double> joint_values;
    ur5e_mgi.getCurrentState()->copyJointGroupPositions(ur5e_mgi.getCurrentState()->getRobotModel()->getJointModelGroup(ur5e_mgi.getName()), joint_values);

    for (std::size_t i = 0; i < 6; ++i)
    {
    ROS_INFO("Joint %ld: %f", i, joint_values[i]);
    }

    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("wrist_3_link");

    /* Print end-effector pose. Remember that this is in the model frame */
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
    ros::shutdown();
}