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

#include <std_msgs/Bool.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "cable_grasp");
    ros::NodeHandle nh;

    ros::start();

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // ros::Publisher isGrasped_pub = nh.advertise<std_msgs::Bool>("grasped_info", 1000);

    bool isFirst_mass_grasped = false;
    bool isLast_mass_grasped = false;


    static const std::string UR5_PLANNING_GROUP = "ur5_arm";
    static const std::string UR5E_PLANNING_GROUP = "ur5e_arm";
    static const std::string UR5_HAND_PLANNING_GROUP = "ur5_hand";
    static const std::string UR5E_HAND_PLANNING_GROUP = "ur5e_hand";


    moveit::planning_interface::MoveGroupInterface ur5_move_group_interface(UR5_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface ur5_hand_move_group_interface(UR5_HAND_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface ur5e_move_group_interface(UR5E_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface ur5e_hand_move_group_interface(UR5E_HAND_PLANNING_GROUP);


    const robot_state::JointModelGroup* ur5_joint_model_group = ur5_move_group_interface.getCurrentState()->getJointModelGroup(UR5_PLANNING_GROUP);
    const robot_state::JointModelGroup* ur5e_joint_model_group = ur5e_move_group_interface.getCurrentState()->getJointModelGroup(UR5E_PLANNING_GROUP);
    const robot_state::JointModelGroup* ur5_hand_joint_model_group = ur5_hand_move_group_interface.getCurrentState()->getJointModelGroup(UR5_HAND_PLANNING_GROUP);
    const robot_state::JointModelGroup* ur5e_hand_joint_model_group = ur5e_hand_move_group_interface.getCurrentState()->getJointModelGroup(UR5E_HAND_PLANNING_GROUP);

    ur5e_move_group_interface.allowReplanning(true); 
    ur5e_move_group_interface.setNumPlanningAttempts(10); 

    ur5e_hand_move_group_interface.allowReplanning(true); 
    ur5e_hand_move_group_interface.setNumPlanningAttempts(10);

    ROS_INFO_NAMED("PLANNING","START PLANNING\n\n\n");

    moveit::core::RobotStatePtr ur5e_current_state = ur5e_move_group_interface.getCurrentState();
    std::vector<double> ur5e_joint_group_positions;
    ur5e_current_state->copyJointGroupPositions(ur5e_joint_model_group, ur5e_joint_group_positions);

    moveit::core::RobotStatePtr ur5e_hand_current_state = ur5e_hand_move_group_interface.getCurrentState();
    std::vector<double> ur5e_hand_joint_group_positions;
    ur5e_hand_current_state->copyJointGroupPositions(ur5e_hand_joint_model_group, ur5e_hand_joint_group_positions);

    moveit::core::RobotStatePtr ur5_current_state = ur5_move_group_interface.getCurrentState();
    std::vector<double> ur5_joint_group_positions;
    ur5_current_state->copyJointGroupPositions(ur5_joint_model_group, ur5_joint_group_positions);

    moveit::core::RobotStatePtr ur5_hand_current_state = ur5_hand_move_group_interface.getCurrentState();
    std::vector<double> ur5_hand_joint_group_positions;
    ur5_hand_current_state->copyJointGroupPositions(ur5_hand_joint_model_group, ur5_hand_joint_group_positions);



    ROS_INFO_NAMED("PLANNING", "Planning frame: %s\n", ur5e_move_group_interface.getPlanningFrame().c_str());
    ROS_INFO_NAMED("PLANNING", "End effector link: %s\n", ur5e_move_group_interface.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("PLANNING", "Reference frame: %s\n", ur5e_move_group_interface.getPoseReferenceFrame().c_str());
    std::cout<<"\n\n\n\n"<<std::endl;


    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //---------------------COLLISION OBJECTS--------------------------//
    //Adding ground to the planning scene

    moveit_msgs::CollisionObject ground;
    ground.header.frame_id = ur5e_move_group_interface.getPlanningFrame();
    ground.id = "ground";
    shape_msgs::SolidPrimitive primitive_ground;
    primitive_ground.type = primitive_ground.BOX;
    primitive_ground.dimensions.resize(3);
    primitive_ground.dimensions[primitive_ground.BOX_X] = 10.5;
    primitive_ground.dimensions[primitive_ground.BOX_Y] = 10.5;
    primitive_ground.dimensions[primitive_ground.BOX_Z] = 0.01;

    
    geometry_msgs::Pose ground_pose;
    ground_pose.orientation.w = 1.0;
    ground_pose.position.x = 0.0;
    ground_pose.position.y = 0.0;
    ground_pose.position.z = 0.0;

    ground.primitives.push_back(primitive_ground);
    ground.primitive_poses.push_back(ground_pose);
    ground.operation = ground.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(ground);

    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);



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

    collision_objects.push_back(ground2);

    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);


    //-------------------------------PLANNING--------------------------------//

    geometry_msgs::PoseStamped current_ur5_arm_pose;
    geometry_msgs::PoseStamped current_ur5e_arm_pose;
    geometry_msgs::PoseStamped target_ur5_arm_pose;
    geometry_msgs::PoseStamped target_ur5e_arm_pose;


    std::vector<geometry_msgs::Pose> ur5_waypoints;
    std::vector<geometry_msgs::Pose> ur5e_waypoints;
    moveit_msgs::RobotTrajectory ur5_trajectory;
    moveit_msgs::RobotTrajectory ur5e_trajectory;


    //-------------------------------HAND OPENING-------------------------------//

    ROS_INFO_NAMED("SINGLE HAND PLANNING","Open hand\n\n\n");
    ur5e_hand_joint_group_positions[0] = 0.0225;
    ur5e_hand_joint_group_positions[1] = 0.045;
    ur5e_hand_move_group_interface.setJointValueTarget(ur5e_hand_joint_group_positions);

    ur5_hand_joint_group_positions[0] = 0.0225;
    ur5_hand_joint_group_positions[1] = 0.045;
    ur5_hand_move_group_interface.setJointValueTarget(ur5_hand_joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan ur5_plan;
    moveit::planning_interface::MoveGroupInterface::Plan ur5e_plan;
    moveit::planning_interface::MoveGroupInterface::Plan ur5_hand_plan;
    moveit::planning_interface::MoveGroupInterface::Plan ur5e_hand_plan;

    bool ur5_success = false;
    bool ur5e_success = false;
    bool ur5_hand_success = false;
    bool ur5e_hand_success = false;

    ur5e_hand_success = (ur5e_hand_move_group_interface.plan(ur5e_hand_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("PLANNING", "Visualizing plan 4 (pose goal) %s\n\n", ur5e_hand_success ? "" : "FAILED");

    ur5_hand_success = (ur5_hand_move_group_interface.plan(ur5_hand_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("PLANNING", "Visualizing plan 4 (pose goal) %s\n\n", ur5_hand_success ? "" : "FAILED");

    if (ur5e_hand_success){
        ur5e_hand_move_group_interface.move();
    }

    if (ur5_hand_success){
        ur5_hand_move_group_interface.move();
    }

    //----------------------------ARM MOVEMENT-------------------------------//

    // ROS_INFO_NAMED("SINGLE ARM PLANNING","Planning to pose goal\n\n\n");
    // current_ur5e_arm_pose = ur5e_move_group_interface.getCurrentPose();
    // target_ur5e_arm_pose = current_ur5e_arm_pose;

    // target_ur5e_arm_pose.pose.position.x = -0.75;
    // target_ur5e_arm_pose.pose.position.y = 0;
    // target_ur5e_arm_pose.pose.position.z = 0.3;


    // tf2::Quaternion quat_tf;
    // tf2::convert(current_ur5e_arm_pose.pose.orientation, quat_tf);
    // tf2::Matrix3x3 m(quat_tf);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);
    // std::cout << "\n\nRoll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << "\n\n\n\n" << std::endl;

    // //Roll: -3.00979, Pitch: -0.120923, Yaw: 2.35672


    // tf2::Quaternion q;
    // geometry_msgs::Quaternion quat_msg;
    // q.setRPY( -3.14, 0, -3.14 ); 
    // q.normalize();
    // quat_msg = tf2::toMsg(q);
    // target_ur5e_arm_pose.pose.orientation.x = quat_msg.x;
    // target_ur5e_arm_pose.pose.orientation.y = quat_msg.y;
    // target_ur5e_arm_pose.pose.orientation.z = quat_msg.z;
    // target_ur5e_arm_pose.pose.orientation.w = quat_msg.w;
    // ur5e_move_group_interface.setPoseTarget(target_ur5e_arm_pose);

    //-------------------PROVAAAAAAAAAAAAAAAAAAAA---------------------------//

    ROS_INFO_NAMED("SINGLE ARM PLANNING","Planning to pose goal\n\n\n");
    current_ur5e_arm_pose = ur5e_move_group_interface.getCurrentPose();
    target_ur5e_arm_pose = current_ur5e_arm_pose;

    target_ur5e_arm_pose.pose.position.x = -0.6;
    target_ur5e_arm_pose.pose.position.y = 0.2;
    target_ur5e_arm_pose.pose.position.z = 0.1;


    tf2::Quaternion q;
    geometry_msgs::Quaternion quat_msg;
    q.setRPY( 0, 0, 0 ); 
    q.normalize();
    quat_msg = tf2::toMsg(q);

    // target_ur5e_arm_pose.pose.orientation.x = quat_msg.x;
    // target_ur5e_arm_pose.pose.orientation.y = quat_msg.y;
    // target_ur5e_arm_pose.pose.orientation.z = quat_msg.z;
    // target_ur5e_arm_pose.pose.orientation.w = quat_msg.w;


    ur5e_move_group_interface.setPoseTarget(target_ur5e_arm_pose);

    ur5e_success = (ur5e_move_group_interface.plan(ur5e_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("PLANNING", "Visualizing plan 2 (pose goal) %s\n\n", ur5e_success ? "" : "FAILED");

    if (ur5e_success){
        ur5e_move_group_interface.move();
    }

    // ros::Duration(5.0).sleep();
    //-----------------------------WRIST ROTATION ----------------------------//
    current_ur5e_arm_pose = ur5e_move_group_interface.getCurrentPose();

    tf2::Quaternion quat_tf;
    tf2::convert(current_ur5e_arm_pose.pose.orientation, quat_tf);
    tf2::Matrix3x3 m(quat_tf);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    std::cout << "\n\nRoll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << "\n\n\n\n" << std::endl;

    //Roll: -3.00536, Pitch: -0.123153, Yaw: 2.3563

    target_ur5e_arm_pose = current_ur5e_arm_pose;
    q.setRPY( -3.14, 0, 1.57 ); 
    q.normalize();
    quat_msg = tf2::toMsg(q);
    target_ur5e_arm_pose.pose.orientation.x = quat_msg.x;
    target_ur5e_arm_pose.pose.orientation.y = quat_msg.y;
    target_ur5e_arm_pose.pose.orientation.z = quat_msg.z;
    target_ur5e_arm_pose.pose.orientation.w = quat_msg.w;

    ur5e_move_group_interface.setPoseTarget(target_ur5e_arm_pose);

    ur5e_success = (ur5e_move_group_interface.plan(ur5e_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("PLANNING", "Visualizing plan 2 (pose goal) %s\n\n", ur5e_success ? "" : "FAILED");

    if (ur5e_success){
        ur5e_move_group_interface.move();
    }

    // ros::Duration(5.0).sleep();

    //----------------------------CARTESIAN PATH-------------------------------//

    current_ur5e_arm_pose = ur5e_move_group_interface.getCurrentPose();
    target_ur5e_arm_pose = current_ur5e_arm_pose;
    target_ur5e_arm_pose.pose.position.z -= 0.05;
    ur5e_waypoints.push_back(target_ur5e_arm_pose.pose);

    ur5e_move_group_interface.computeCartesianPath(ur5e_waypoints, 0.01, 0.0, ur5e_trajectory); // 0.01 eef_step, 0.0 jump_threshold
    ur5e_waypoints.clear();

    ur5e_move_group_interface.execute(ur5e_trajectory);

    //----------------------------HAND CLOSE-------------------------------//

    ROS_INFO_NAMED("SINGLE HAND PLANNING","Close hand\n\n\n");
    ur5e_hand_joint_group_positions[0] = 0.014;
    ur5e_hand_joint_group_positions[1] = 0.028;
    ur5e_hand_move_group_interface.setJointValueTarget(ur5e_hand_joint_group_positions);

    ur5e_hand_success = (ur5e_hand_move_group_interface.plan(ur5e_hand_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("PLANNING", "Visualizing plan (pose goal) %s\n\n", ur5e_hand_success ? "" : "FAILED");

    if (ur5e_hand_success){
        ur5e_hand_move_group_interface.move();
        bool isGrasped = true;
    }

    ros::Duration(2.0).sleep();

    //----------------------------FINE-PROVA-------------------------------//

    //ur5_ARM

    current_ur5_arm_pose = ur5_move_group_interface.getCurrentPose();
    target_ur5_arm_pose = current_ur5_arm_pose;

    target_ur5_arm_pose.pose.position.x = 0.85;
    target_ur5_arm_pose.pose.position.y = 0;
    target_ur5_arm_pose.pose.position.z = 0.3;

    tf2::Quaternion quat2_tf;
    tf2::convert(current_ur5_arm_pose.pose.orientation, quat2_tf);
    tf2::Matrix3x3 m2(quat2_tf);
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
    ur5e_move_group_interface.setPlanningTime(10.0);


    ur5e_success = (ur5e_move_group_interface.plan(ur5e_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("PLANNING", "Visualizing plan 2 (pose goal) %s\n\n", ur5e_success ? "" : "FAILED");

    if (ur5e_success){
        ur5e_move_group_interface.move();
    }

    ur5_success = (ur5_move_group_interface.plan(ur5_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("PLANNING", "Visualizing plan 2 (pose goal) %s\n\n", ur5_success ? "" : "FAILED");

    if (ur5_success){
        ur5_move_group_interface.move();
    }


    //----------------------------CARTESIAN PATH-------------------------------//

    current_ur5e_arm_pose = ur5e_move_group_interface.getCurrentPose();
    target_ur5e_arm_pose = current_ur5e_arm_pose;
    target_ur5e_arm_pose.pose.position.x += 0.05;
    ur5e_waypoints.push_back(target_ur5e_arm_pose.pose);

    ur5e_move_group_interface.computeCartesianPath(ur5e_waypoints, 0.01, 0.0, ur5e_trajectory); // 0.01 eef_step, 0.0 jump_threshold
    ur5e_waypoints.clear();


    current_ur5_arm_pose = ur5_move_group_interface.getCurrentPose();
    target_ur5_arm_pose = current_ur5_arm_pose;
    target_ur5_arm_pose.pose.position.x -= 0.05;
    ur5_waypoints.push_back(target_ur5_arm_pose.pose);

    ur5_move_group_interface.computeCartesianPath(ur5_waypoints, 0.01, 0.0, ur5_trajectory); // 0.01 eef_step, 0.0 jump_threshold
    ur5_waypoints.clear();

    ur5e_move_group_interface.execute(ur5e_trajectory);
    ur5_move_group_interface.execute(ur5_trajectory);

    //----------------------------HAND CLOSE-------------------------------//

    ROS_INFO_NAMED("SINGLE HAND PLANNING","Close hand\n\n\n");
    ur5e_hand_joint_group_positions[0] = 0.0015;
    ur5e_hand_joint_group_positions[1] = 0.003;
    ur5e_hand_move_group_interface.setJointValueTarget(ur5e_hand_joint_group_positions);

    ur5_hand_joint_group_positions[0] = 0.0015;
    ur5_hand_joint_group_positions[1] = 0.003;
    ur5_hand_move_group_interface.setJointValueTarget(ur5_hand_joint_group_positions);

    ur5e_hand_success = (ur5e_hand_move_group_interface.plan(ur5e_hand_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("PLANNING", "Visualizing plan (pose goal) %s\n\n", ur5e_hand_success ? "" : "FAILED");

    ur5_hand_success = (ur5_hand_move_group_interface.plan(ur5_hand_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("PLANNING", "Visualizing plan (pose goal) %s\n\n", ur5_hand_success ? "" : "FAILED");

    if (ur5e_hand_success){
        ur5e_hand_move_group_interface.move();
        isFirst_mass_grasped= true;
    }

    if (ur5_hand_success){
        ur5_hand_move_group_interface.move();
        isLast_mass_grasped = true;
    }
    
    
    

 

    ros::Duration(2.0).sleep();

    //----------------------------CARTESIAN PATH-------------------------------//

    // current_ur5e_arm_pose = ur5e_move_group_interface.getCurrentPose();
    // target_ur5e_arm_pose = current_ur5e_arm_pose;
    // target_ur5e_arm_pose.pose.position.x -= 0.05;
    // ur5e_waypoints.push_back(target_ur5e_arm_pose.pose);

    // ur5e_move_group_interface.computeCartesianPath(ur5e_waypoints, 0.01, 0.0, ur5e_trajectory); // 0.01 eef_step, 0.0 jump_threshold
    // ur5e_waypoints.clear();


    // current_ur5_arm_pose = ur5_move_group_interface.getCurrentPose();
    // target_ur5_arm_pose = current_ur5_arm_pose;
    // target_ur5_arm_pose.pose.position.x += 0.05;
    // ur5_waypoints.push_back(target_ur5_arm_pose.pose);

    // ur5_move_group_interface.computeCartesianPath(ur5_waypoints, 0.01, 0.0, ur5_trajectory); // 0.01 eef_step, 0.0 jump_threshold
    // ur5_waypoints.clear();

    // ur5e_move_group_interface.execute(ur5e_trajectory);
    // ur5_move_group_interface.execute(ur5_trajectory);


    //----------------------------CARTESIAN PATH-------------------------------//

    // current_ur5e_arm_pose = ur5e_move_group_interface.getCurrentPose();
    // target_ur5e_arm_pose = current_ur5e_arm_pose;
    // target_ur5e_arm_pose.pose.position.z -= 0.25;
    // ur5e_waypoints.push_back(target_ur5e_arm_pose.pose);

    // ur5e_move_group_interface.computeCartesianPath(ur5e_waypoints, 0.01, 0.0, ur5e_trajectory); // 0.01 eef_step, 0.0 jump_threshold
    // ur5e_waypoints.clear();

    // ur5e_move_group_interface.execute(ur5e_trajectory);

    //-------------------------------HAND OPENING-------------------------------//

    // ROS_INFO_NAMED("SINGLE HAND PLANNING","Open hand\n\n\n");
    // ur5e_hand_joint_group_positions[0] = 0.0225;
    // ur5e_hand_joint_group_positions[1] = 0.045;
    // ur5e_hand_move_group_interface.setJointValueTarget(ur5e_hand_joint_group_positions);

    // ur5e_hand_success = (ur5e_hand_move_group_interface.plan(ur5e_hand_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("PLANNING", "Visualizing plan (pose goal) %s\n\n", ur5e_hand_success ? "" : "FAILED");

    // if (ur5e_hand_success){
    //     ur5e_hand_move_group_interface.move();
    // }


    current_ur5e_arm_pose = ur5e_move_group_interface.getCurrentPose();
    target_ur5e_arm_pose = current_ur5e_arm_pose;
    target_ur5e_arm_pose.pose.position.x -= 0.55;
    ur5e_waypoints.push_back(target_ur5e_arm_pose.pose);

    ur5e_move_group_interface.computeCartesianPath(ur5e_waypoints, 0.01, 0.0, ur5e_trajectory); // 0.01 eef_step, 0.0 jump_threshold
    ur5e_waypoints.clear();
    ur5e_move_group_interface.execute(ur5e_trajectory);

    // ROS_INFO_NAMED("SINGLE HAND PLANNING","Open hand\n\n\n");
    // ur5e_hand_joint_group_positions[0] = 0.0225;
    // ur5e_hand_joint_group_positions[1] = 0.045;
    // ur5e_hand_move_group_interface.setJointValueTarget(ur5e_hand_joint_group_positions);


    // ur5e_hand_success = (ur5e_hand_move_group_interface.plan(ur5e_hand_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("PLANNING", "Visualizing plan 4 (pose goal) %s\n\n", ur5e_hand_success ? "" : "FAILED");

    // if (ur5e_hand_success){
    //     ur5e_hand_move_group_interface.move();
    // }


    ros::shutdown();
    return 0;
}