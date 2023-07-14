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


int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "master_controller");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2); 
    spinner.start();

    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ur5_arm_controller/command", 100);

    ros::ServiceClient grasp_client = nh.serviceClient<cable_model::CableMsg>("set_cable_grasp");
    cable_model::CableMsg srv;

    static const std::string UR5_PLANNING_GROUP = "ur5_arm";
    static const std::string UR5_HAND_PLANNING_GROUP = "ur5_hand";

    moveit::planning_interface::MoveGroupInterface ur5_move_group_interface(UR5_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface ur5_hand_move_group_interface(UR5_HAND_PLANNING_GROUP);

    const robot_state::JointModelGroup* ur5_joint_model_group = ur5_move_group_interface.getCurrentState()->getJointModelGroup(UR5_PLANNING_GROUP);
    const robot_state::JointModelGroup* ur5_hand_joint_model_group = ur5_hand_move_group_interface.getCurrentState()->getJointModelGroup(UR5_HAND_PLANNING_GROUP);

    ur5_move_group_interface.allowReplanning(true); 
    ur5_move_group_interface.setNumPlanningAttempts(10); 

    ur5_hand_move_group_interface.allowReplanning(true); 
    ur5_hand_move_group_interface.setNumPlanningAttempts(10);


    ROS_INFO_NAMED("PLANNING","START PLANNING\n\n\n");

    moveit::core::RobotStatePtr ur5_current_state = ur5_move_group_interface.getCurrentState();
    std::vector<double> ur5_joint_group_positions;
    ur5_current_state->copyJointGroupPositions(ur5_joint_model_group, ur5_joint_group_positions);

    moveit::core::RobotStatePtr ur5_hand_current_state = ur5_hand_move_group_interface.getCurrentState();
    std::vector<double> ur5_hand_joint_group_positions;
    ur5_hand_current_state->copyJointGroupPositions(ur5_hand_joint_model_group, ur5_hand_joint_group_positions);



    ROS_INFO_NAMED("PLANNING", "Planning frame: %s\n", ur5_move_group_interface.getPlanningFrame().c_str());
    ROS_INFO_NAMED("PLANNING", "End effector link: %s\n", ur5_move_group_interface.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("PLANNING", "Reference frame: %s\n", ur5_move_group_interface.getPoseReferenceFrame().c_str());
    std::cout<<"\n\n\n\n"<<std::endl;

    //moveit has problems when 2 trajectories are executed at the same time, even if they are for different groups
    ros::Duration(10.0).sleep();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //---------------------COLLISION OBJECTS--------------------------//
        //Adding ground to the planning scene


    moveit_msgs::CollisionObject ground2;
    ground2.header.frame_id = ur5_move_group_interface.getPlanningFrame();
    ground2.id = "ground2";
    shape_msgs::SolidPrimitive primitive_ground2;
    primitive_ground2.type = primitive_ground2.BOX;
    primitive_ground2.dimensions.resize(3);
    primitive_ground2.dimensions[primitive_ground2.BOX_X] = 10.5;
    primitive_ground2.dimensions[primitive_ground2.BOX_Y] = 10.5;
    primitive_ground2.dimensions[primitive_ground2.BOX_Z] = 0.01;

    
    geometry_msgs::Pose ground_pose2;
    ground_pose2.orientation.w = 1.0;
    ground_pose2.position.x = 0.0;
    ground_pose2.position.y = 0.0;
    ground_pose2.position.z = 0.0;

    ground2.primitives.push_back(primitive_ground2);
    ground2.primitive_poses.push_back(ground_pose2);
    ground2.operation = ground2.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(ground2);

    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    //-------------------------------PLANNING--------------------------------//
    geometry_msgs::PoseStamped current_ur5_arm_pose;
    geometry_msgs::PoseStamped target_ur5_arm_pose;

    std::vector<geometry_msgs::Pose> ur5_waypoints;
    moveit_msgs::RobotTrajectory ur5_trajectory;

    //-------------------------------HAND OPENING-------------------------------//
    ROS_INFO_NAMED("SINGLE HAND PLANNING","Open hand\n\n\n");

    ur5_hand_joint_group_positions[0] = 0.0225;
    ur5_hand_joint_group_positions[1] = 0.045;
    ur5_hand_move_group_interface.setJointValueTarget(ur5_hand_joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan ur5_plan;
    moveit::planning_interface::MoveGroupInterface::Plan ur5_hand_plan;

    bool ur5_success = false;
    bool ur5_hand_success = false;

    ur5_hand_success = (ur5_hand_move_group_interface.plan(ur5_hand_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("PLANNING", "Visualizing plan 4 (pose goal) %s\n\n", ur5_hand_success ? "" : "FAILED");

    if (ur5_hand_success){
        ur5_hand_move_group_interface.move();
    }    

    //----------------------------ARM MOVEMENT-------------------------------//

    ROS_INFO_NAMED("SINGLE ARM PLANNING","Planning to pose goal\n\n\n");

    current_ur5_arm_pose = ur5_move_group_interface.getCurrentPose();
    target_ur5_arm_pose = current_ur5_arm_pose;

    target_ur5_arm_pose.pose.position.x = 0.85;
    target_ur5_arm_pose.pose.position.y = 0;
    target_ur5_arm_pose.pose.position.z = 0.3;

    tf2::Quaternion quat2_tf;
    tf2::convert(current_ur5_arm_pose.pose.orientation, quat2_tf);
    tf2::Matrix3x3 m2(quat2_tf);
    double roll, pitch, yaw;
    m2.getRPY(roll, pitch, yaw);
    std::cout << "\n\nRoll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << "\n\n\n\n" << std::endl;

    //Roll: -3.00701, Pitch: -0.12877, Yaw: -0.789105

    tf2::Quaternion q2;
    geometry_msgs::Quaternion quat_msg2;
    q2.setRPY( -3.14, 0, -0 );
    q2.normalize();
    quat_msg2 = tf2::toMsg(q2);
    target_ur5_arm_pose.pose.orientation.x = quat_msg2.x;
    target_ur5_arm_pose.pose.orientation.y = quat_msg2.y;
    target_ur5_arm_pose.pose.orientation.z = quat_msg2.z;
    target_ur5_arm_pose.pose.orientation.w = quat_msg2.w;
    ur5_move_group_interface.setPoseTarget(target_ur5_arm_pose);

    ur5_move_group_interface.setPlanningTime(10.0);

    ur5_success = (ur5_move_group_interface.plan(ur5_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("PLANNING", "Visualizing plan 2 (pose goal) %s\n\n", ur5_success ? "" : "FAILED");

    if (ur5_success){
        ur5_move_group_interface.move();
    }


    //----------------------------CARTESIAN PATH-------------------------------//
    current_ur5_arm_pose = ur5_move_group_interface.getCurrentPose();
    target_ur5_arm_pose = current_ur5_arm_pose;
    target_ur5_arm_pose.pose.position.x -= 0.05;
    ur5_waypoints.push_back(target_ur5_arm_pose.pose);

    ur5_move_group_interface.computeCartesianPath(ur5_waypoints, 0.01, 0.0, ur5_trajectory); // 0.01 eef_step, 0.0 jump_threshold
    ur5_waypoints.clear();

    ur5_move_group_interface.execute(ur5_trajectory);

    //----------------------------HAND CLOSE-------------------------------//

    ROS_INFO_NAMED("SINGLE HAND PLANNING","Close hand\n\n\n");
    ur5_hand_joint_group_positions[0] = 0.0015;
    ur5_hand_joint_group_positions[1] = 0.003;
    ur5_hand_move_group_interface.setJointValueTarget(ur5_hand_joint_group_positions);

    ur5_hand_success = (ur5_hand_move_group_interface.plan(ur5_hand_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("PLANNING", "Visualizing plan (pose goal) %s\n\n", ur5_hand_success ? "" : "FAILED");

    if (ur5_hand_success){
        ur5_hand_move_group_interface.move();
        // isLast_mass_grasped = true;
        srv.request.set_massN_grasped = false;
        //srv.request.set_mass0_grasped = false;
    }
    
    if (grasp_client.call(srv))
    {
        ROS_INFO("mass N: %d", srv.response.is_massN_grasped);
    }
    else
    {
        ROS_ERROR("Failed to call service set_cable_grasp");
        // return 0;
    }
    ros::Duration(1.0).sleep();

    //----------------------------ARM MOVEMENT-------------------------------//
    // Set the scaling factors for velocity and acceleration
    ur5_move_group_interface.setMaxVelocityScalingFactor(0.01); // Reduce the maximum velocity to 1%
    ur5_move_group_interface.setMaxAccelerationScalingFactor(0.01); // Reduce the maximum acceleration to 10%

    current_ur5_arm_pose = ur5_move_group_interface.getCurrentPose();
    target_ur5_arm_pose = current_ur5_arm_pose;
    target_ur5_arm_pose.pose.position.x += 0.18;
    ur5_waypoints.push_back(target_ur5_arm_pose.pose);

    bool success = ur5_move_group_interface.computeCartesianPath(ur5_waypoints, 0.01, 0.0, ur5_trajectory); // 0.01 eef_step, 0.0 jump_threshold
    // ur5_waypoints.clear();
    // ur5_move_group_interface.execute(ur5_trajectory);



    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // ur5_trajectory.joint_trajectory.points[0].time_from_start = ros::Duration(2.0);
    plan.trajectory_ = ur5_trajectory;

    ROS_INFO_NAMED("SINGLE ARM PLANNING","STARTING CARTESIAN MOVEMENT\n\n\n");
    ur5_move_group_interface.execute(plan);


    ros::shutdown();
    return 0;
}
