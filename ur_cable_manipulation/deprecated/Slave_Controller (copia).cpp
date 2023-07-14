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
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"

#include <iostream>
#include <geometry_msgs/Wrench.h>


#include <ur_cable_manipulation/Jacobian.hpp>
#include <ur_cable_manipulation/Kinematics.hpp>
#include <ur_cable_manipulation/PIDController.hpp>
#include <ur_cable_manipulation/SensorReader.hpp>
#include <ur_cable_manipulation/AdmittanceControl.hpp>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "slave_controller");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(3); 
    spinner.start();
    ros::Rate rate(50);

    SensorReader sensor_reader(rate);

    //spins two threads, one for the subscriber and one for the publisher
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ur5e_arm_controller/command", 100);
    ros::Subscriber sub = nh.subscribe("/ft_sensor", 100, &SensorReader::FT_sensor_Reading_Callback, &sensor_reader);

    ros::ServiceClient grasp_client = nh.serviceClient<cable_model::CableMsg>("set_cable_grasp");
    cable_model::CableMsg srv;

    //------------------------MOVEIT PLANNING-------------------------------------------//

    static const std::string UR5E_PLANNING_GROUP = "ur5e_arm";
    static const std::string UR5E_HAND_PLANNING_GROUP = "ur5e_hand";

    moveit::planning_interface::MoveGroupInterface ur5e_move_group_interface(UR5E_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface ur5e_hand_move_group_interface(UR5E_HAND_PLANNING_GROUP);

    const robot_state::JointModelGroup* ur5e_joint_model_group = ur5e_move_group_interface.getCurrentState()->getJointModelGroup(UR5E_PLANNING_GROUP);
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

    ROS_INFO_NAMED("PLANNING", "Planning frame: %s\n", ur5e_move_group_interface.getPlanningFrame().c_str());  //world
    ROS_INFO_NAMED("PLANNING", "End effector link: %s\n", ur5e_move_group_interface.getEndEffectorLink().c_str());  //ur5e_finger_tip and ur5_finger_tip
    ROS_INFO_NAMED("PLANNING", "Reference frame: %s\n", ur5e_move_group_interface.getPoseReferenceFrame().c_str());  //world


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

    ROS_INFO_NAMED("tutorial", "Add object into the world to avoid  ground collision");
    planning_scene_interface.addCollisionObjects(collision_objects);

    //-------------------------------PLANNING--------------------------------//

    geometry_msgs::PoseStamped current_ur5e_arm_pose;
    geometry_msgs::PoseStamped target_ur5e_arm_pose;

    std::vector<geometry_msgs::Pose> ur5e_waypoints;
    moveit_msgs::RobotTrajectory ur5e_trajectory;

    //-------------------------------HAND OPENING-------------------------------//

    ROS_INFO_NAMED("SINGLE HAND PLANNING","Open hand\n\n\n");
    ur5e_hand_joint_group_positions[0] = 0.0225;
    ur5e_hand_joint_group_positions[1] = 0.045;
    ur5e_hand_move_group_interface.setJointValueTarget(ur5e_hand_joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan ur5e_plan;
    moveit::planning_interface::MoveGroupInterface::Plan ur5e_hand_plan;

    bool ur5e_success = false;
    bool ur5e_hand_success = false;

    ur5e_hand_success = (ur5e_hand_move_group_interface.plan(ur5e_hand_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("PLANNING", "Visualizing hand open plan %s\n\n", ur5e_hand_success ? "" : "FAILED");


    if (ur5e_hand_success)
        ur5e_hand_move_group_interface.move();
        
            
    //----------------------------ARM MOVEMENT-------------------------------//

    ROS_INFO_NAMED("SINGLE ARM PLANNING","Planning to pose goal\n\n\n");
    current_ur5e_arm_pose = ur5e_move_group_interface.getCurrentPose();
    target_ur5e_arm_pose = current_ur5e_arm_pose;

    target_ur5e_arm_pose.pose.position.x = -0.75;
    target_ur5e_arm_pose.pose.position.y = 0;
    target_ur5e_arm_pose.pose.position.z = 0.3;

    //Converts geometry_msgs::quaternion to tf2::quaternion
    tf2::Quaternion quat_tf;
    tf2::convert(current_ur5e_arm_pose.pose.orientation, quat_tf);
    //implement rotation matrix from quaternion
    tf2::Matrix3x3 m(quat_tf);
    double roll, pitch, yaw;
    //get roll pitch yaw from rotation matrix
    m.getRPY(roll, pitch, yaw);
    // std::cout << "\n\nRoll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << "\n\n\n\n" << std::endl;

    //Roll: -3.00979, Pitch: -0.120923, Yaw: 2.35672


    tf2::Quaternion q;
    geometry_msgs::Quaternion quat_msg;
    q.setRPY( -3.14, 0, -3.14 ); 
    q.normalize();
    quat_msg = tf2::toMsg(q);
    target_ur5e_arm_pose.pose.orientation.x = quat_msg.x;
    target_ur5e_arm_pose.pose.orientation.y = quat_msg.y;
    target_ur5e_arm_pose.pose.orientation.z = quat_msg.z;
    target_ur5e_arm_pose.pose.orientation.w = quat_msg.w;
    ur5e_move_group_interface.setPoseTarget(target_ur5e_arm_pose);

    ur5e_move_group_interface.setPlanningTime(10.0);
    ur5e_success = (ur5e_move_group_interface.plan(ur5e_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("PLANNING", "Visualizing cable approach plan in joint space (pose goal) %s\n\n", ur5e_success ? "" : "FAILED");

    if (ur5e_success)
        ur5e_move_group_interface.move();
    
    //----------------------------CARTESIAN PATH-------------------------------//

    current_ur5e_arm_pose = ur5e_move_group_interface.getCurrentPose();
    target_ur5e_arm_pose = current_ur5e_arm_pose;
    target_ur5e_arm_pose.pose.position.x += 0.05;
    ur5e_waypoints.push_back(target_ur5e_arm_pose.pose);

    ur5e_move_group_interface.computeCartesianPath(ur5e_waypoints, 0.01, 0.0, ur5e_trajectory); // 0.01 eef_step, 0.0 jump_threshold
    ur5e_waypoints.clear();

    ur5e_move_group_interface.execute(ur5e_trajectory);


    //-------------------------------SENSOR CALIBRATION-------------------------------//

    ROS_INFO_NAMED("SENSOR CALIBRATION","Calibrating sensor\n\n\n");
    sensor_reader.startCalibration();
    std::cout << "Calibration started\n" << std::endl;
    std::vector<double> sensor_bias = sensor_reader.calibrateSensor();
    for (int i = 0; i < sensor_bias.size(); i++)
        std::cout << "Bias " << i << ": " << sensor_bias[i] << "\n"<<std::endl;

    sensor_reader.setSensorCalibrated(true);
    sensor_reader.clearReadings();

    //----------------------------HAND CLOSE-------------------------------//

    ROS_INFO_NAMED("SINGLE HAND PLANNING","Close hand\n\n\n");
    ur5e_hand_joint_group_positions[0] = 0.0016;
    ur5e_hand_joint_group_positions[1] = 0.0032;
    ur5e_hand_move_group_interface.setJointValueTarget(ur5e_hand_joint_group_positions);

    ur5e_hand_success = (ur5e_hand_move_group_interface.plan(ur5e_hand_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("PLANNING", "Visualizing hand close plan (pose goal) %s\n\n", ur5e_hand_success ? "" : "FAILED");

    //calls service used to communicate to the cable that the particle is not static anymore
    if (ur5e_hand_success){
        ur5e_hand_move_group_interface.move();
        srv.request.set_mass0_grasped = false;
        srv.request.set_massN_grasped = true;
    }

    if (grasp_client.call(srv))
    {
        ROS_INFO("mass 0: %d", srv.response.is_mass0_grasped);
    }
    else
    {
        ROS_ERROR("Failed to call service set_cable_grasp");
    }
    ros::Duration(1.0).sleep();

    //----------------------------FEEDBACK CONTROL START-------------------------------//

    //Force expressed in the sensor frame (ur5e_wrist_3_link)
    // std::vector<double> error_force(3);
    // std::vector<double> error_torque(3);
    // std::vector<double> desired_force = sensor_reader.getDesiredForce();
    // std::vector<double> desired_torque = sensor_reader.getDesiredTorque();

    // std::vector<double> k_inv = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};


    // std::vector<double> filtered_unbiased_force(3);
    // std::vector<double> filtered_unbiased_torque(3);
    std::vector<double> joint_goal(6);
    std::vector<double> joint_vel_goal(6);
    std::vector<double> joint_acc_goal(6);
    // joint_vel_goal = {1.3,1.3,1.3,1.3,1.3,1.3};
    // joint_acc_goal = {1.3,1.3,1.3,1.3,1.3,1.3};


    // Eigen::VectorXd joint_goal_eigen(6);
    Eigen::VectorXd pose_goal_eigen(6);
    Eigen::VectorXd pose_goal_quat(6);

    std::vector<double> bl_force(3);
    std::vector<double> bl_torque(3);
    std::vector<double> bl_wrench(6);

    std::vector<double> wrist_wrt_bl(6);
    std::vector<double> wrist_wrt_bl_quat(6);


    // ros::Rate rate(50);
    AdmittanceController admittance_controller(rate);
    ur_kinematics::Kinematics kinematics("ur5e");

    // Define an 8x6 matrix of doubles
    double q_sols[8*6];
    double T[16];
    int num_sols;
    int sol_selected;

    int i = 0;
    

    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    // ROS_INFO("Time: %f\n", traj.header.stamp.toSec());
    traj.header.frame_id = "ur5e_base_link";

    traj.joint_names.resize(6);

    traj.joint_names[0] = "ur5e_shoulder_pan_joint";
    traj.joint_names[1] = "ur5e_shoulder_lift_joint";
    traj.joint_names[2] = "ur5e_elbow_joint";
    traj.joint_names[3] = "ur5e_wrist_1_joint";
    traj.joint_names[4] = "ur5e_wrist_2_joint";
    traj.joint_names[5] = "ur5e_wrist_3_joint";


    // for (int i=0; i<6; i++){
    //     ROS_INFO("Joint %d: %s\n", i, traj.joint_names[i].c_str()); 
    // }

    // std::vector<double> joint_positions = ur5e_move_group_interface.getCurrentJointValues();
    // std::vector<double> current_end_effectors_pose(6);

    traj.points.resize(1);
    traj.points[0].positions.resize(6);

    // std::vector<double> end_effector_goal(6);


    //initialize the joint goal with the current joint values
    // traj.points[0].positions = ur5e_move_group_interface.getCurrentJointValues();

    
    //----------------------------FEEDBACK CONTROL LOOP-------------------------------//

    while(ros::ok()){ 
        
        std::vector<double> current_joint_values = ur5e_move_group_interface.getCurrentJointValues();
        std::vector<double> pos_joint_values(6);
        // ur5e_move_group_interface.getCurrentState()->copyJointGroupPositions(ur5e_move_group_interface.getCurrentState()->getRobotModel()->getJointModelGroup(ur5e_move_group_interface.getName()), current_joint_values);
        
        //-----compute jacobian-----//
        Eigen::MatrixXd custom_jacobian = Jacobian::getJacobian_ur5e_wrt_bl(current_joint_values[0], current_joint_values[1], current_joint_values[2], current_joint_values[3], current_joint_values[4], current_joint_values[5]);
        // std::cout<<"CUSTOM jacobian: "<<custom_jacobian<<std::endl; 
        Eigen::MatrixXd j_inv = custom_jacobian.inverse();

        //-----compute end effector goal-----//
        //this gives finger tip position wrt world frame
        // current_end_effectors_pose[0] = ur5e_move_group_interface.getCurrentPose().pose.position.x;
        // current_end_effectors_pose[1] = ur5e_move_group_interface.getCurrentPose().pose.position.y;
        // current_end_effectors_pose[2] = ur5e_move_group_interface.getCurrentPose().pose.position.z;

        //------compute force and torque wrt base link------//
        bl_wrench= sensor_reader.getFilteredForce_in_bl();
        std::copy(bl_wrench.begin(), bl_wrench.begin() + 3, bl_force.begin());
        std::copy(bl_wrench.begin() + 3, bl_wrench.end(), bl_torque.begin());

        
        ROS_INFO("Force and torque wrt base link: \n%f, %f, %f, %f, %f, %f\n", bl_force[0], bl_force[1], bl_force[2], bl_torque[0], bl_torque[1], bl_torque[2]);


        for(int i=0; i<3; i++){
            // error_force[i] = desired_force[i] - (sensor_reader.getForceDetected()[i] - sensor_bias[i]);
            // error_torque[i] = desired_torque[i] - (sensor_reader.getTorqueDetected()[i] - sensor_bias[i+3]);

            // filtered_unbiased_force[i] = sensor_reader.getFilteredForce()[i]  ; //- sensor_bias[i];
            // filtered_unbiased_torque[i] = sensor_reader.getFilteredTorque()[i]; //- sensor_bias[i+3];

            // bl_force[i] = bl_wrench[i];
            // bl_torque[i] = bl_wrench[i+3];

            bl_force[i] -= sensor_bias[i];
            bl_torque[i] -= sensor_bias[i+3];

            // error_force[i] = desired_force[i] - (sensor_reader.getFilteredForce()[i] - sensor_bias[i]);
            // error_torque[i] = desired_torque[i] - (sensor_reader.getFilteredTorque()[i] - sensor_bias[i+3]);

            // ROS_INFO("Error force %d: %f\n", i, error_force[i]);
            // ROS_INFO("Error torque %d: %f\n\n", i, error_torque[i]);

            // ROS_INFO("Unbiased Force %d: %f\n", i, sensor_reader.getForceDetected()[i]- sensor_bias[i]);
            // ROS_INFO("Unbiased Torque %d: %f\n", i, sensor_reader.getTorqueDetected()[i]- sensor_bias[i+3]);

            // ROS_INFO("Filtered Force %d: %f\n", i, sensor_reader.getFilteredForce()[i]);
            // ROS_INFO("Filtered Torque %d: %f\n\n\n", i, sensor_reader.getFilteredTorque()[i]);

            // ROS_INFO("Unbiased filtered Force %d: %f\n", i, sensor_reader.getFilteredForce()[i]- sensor_bias[i]);
            // ROS_INFO("Unbiased filtered Torque %d: %f\n\n", i, sensor_reader.getFilteredTorque()[i]- sensor_bias[i+3]);
            
            // current_end_effectors_pose[i+3] = ur5e_move_group_interface.getCurrentRPY()[i];

            // ROS_INFO("Current end effector pose %d: %f\n", i, current_end_effectors_pose[i]);
            // ROS_INFO("Current end effector rot %d: %f\n", i+3, current_end_effectors_pose[i+3]);

            // end_effector_goal[i] = current_end_effectors_pose[i] + k_inv[i]*error_force[i];
            // end_effector_goal[i+3] = current_end_effectors_pose[i+3] + k_inv[i+3]*error_torque[i]; 

            // ROS_INFO("End effector goal %d: %f\n", i, end_effector_goal[i]);
            // ROS_INFO("End effector goal %d: %f\n\n", i+3, end_effector_goal[i+3]);
            // std::cout <<"---"<< std::endl;

        }

        ROS_INFO("UNBIASED Force and torque wrt base link: \n%f, %f, %f, %f, %f, %f\n", bl_force[0], bl_force[1], bl_force[2], bl_torque[0], bl_torque[1], bl_torque[2]);


        //------COMPUTE POSE GOAL VIA ADMITTANCE CONTROL------//

        //find transformation between wrist_3_link and base_link, to get end effector pose wrt base link
        // wrist_wrt_bl = kinematics.tfPose_rpy("ur5e_flange", "ur5e_base_link");
        // wrist_wrt_bl = kinematics.tfPose_rpy("ur5e_wrist_3_link", "ur5e_base_link");
        // wrist_wrt_bl_quat = kinematics.tfPose_quat("ur5e_tool0", "ur5e_base_link");

        


        //compute actual position only when the trajectory is completed
        ROS_INFO("Current joint values: \n%f, %f, %f, %f, %f, %f\n\n", current_joint_values[0], current_joint_values[1], current_joint_values[2], current_joint_values[3], current_joint_values[4], current_joint_values[5]);
        ROS_INFO("Joint goal: \n%f, %f, %f, %f, %f, %f\n\n", traj.points[0].positions[0], traj.points[0].positions[1], traj.points[0].positions[2], traj.points[0].positions[3], traj.points[0].positions[4], traj.points[0].positions[5]);
        // if (current_joint_values == traj.points[0].positions)
        // if (i==0)
            wrist_wrt_bl = kinematics.tfPose_rpy("ur5e_flange", "ur5e_base_link");


        ROS_INFO("Wrist wrt base link (RPY): \n%f, %f, %f, %f, %f, %f\n\n", wrist_wrt_bl[0], wrist_wrt_bl[1], wrist_wrt_bl[2], wrist_wrt_bl[3], wrist_wrt_bl[4], wrist_wrt_bl[5]);
        // ROS_INFO("Wrist wrt base link (QUAT): \n%f, %f, %f, %f, %f, %f, %f\n\n", wrist_wrt_bl_quat[0], wrist_wrt_bl_quat[1], wrist_wrt_bl_quat[2], wrist_wrt_bl_quat[3], wrist_wrt_bl_quat[4], wrist_wrt_bl_quat[5], wrist_wrt_bl_quat[6]);

        try{
            pose_goal_eigen = admittance_controller.computeAdmittance(bl_force, bl_torque, wrist_wrt_bl);
            ROS_INFO("Pose goal (RPY): \n%f, %f, %f, %f, %f, %f\n\n", pose_goal_eigen[0], pose_goal_eigen[1], pose_goal_eigen[2], pose_goal_eigen[3], pose_goal_eigen[4], pose_goal_eigen[5]);
            i++;
            std::cout <<"i: "<< i << std::endl;
        } catch (const Exception& e) {
            std::cout <<"i: "<< i << std::endl;
            std::cerr << e.what() << '\n';
            // for (int i = 0; i < 6; i++)
            // {
            //     pose_goal_eigen[i] = wrist_wrt_bl[i];
            // }
            // ros::Duration(10.0).sleep();
        }

        // Eigen::Matrix3d rotation_matrix;
        // rotation_matrix = Eigen::AngleAxisd( pose_goal_eigen[5], Eigen::Vector3d::UnitZ())
        //             * Eigen::AngleAxisd( pose_goal_eigen[4], Eigen::Vector3d::UnitY())
        //             * Eigen::AngleAxisd( pose_goal_eigen[3], Eigen::Vector3d::UnitX());
        // Output the rotation matrix 
        // std::cout << "Rotation Matrix ZYX:\n" << rotation_matrix << std::endl;


        //transform vector pose into homogeneous matrix
        Eigen::Matrix4d Hom_matrix = kinematics.Vector_To_HomogeneusMatrix(pose_goal_eigen[0], pose_goal_eigen[1], pose_goal_eigen[2], pose_goal_eigen[3], pose_goal_eigen[4], pose_goal_eigen[5]);
        std::cout << "Homogeneous Matrix:\n" << Hom_matrix << "\n" << std::endl;

        Eigen::Matrix4d T_6th_to_ee(4,4);
        T_6th_to_ee <<  1,  0,  0,  0,
                        0, -1,  0,  0,
                        0,  0, -1,  0,
                        0,  0,  0,  1;
        Eigen::MatrixXd end_inverse = T_6th_to_ee.inverse();
    
        // std::cout << "Homogeneous Matrix end inverse:\n" << end_inverse << "\n" << std::endl;

        // Eigen::Matrix4d T_bl_to_0th(4,4);
        // T_bl_to_0th <<  -1, 0,  0,  0,
        //                 0,  -1, 0,  0,
        //                 0,  0,  1,  0,
        //                 0,  0,  0,  1;
        // Eigen::MatrixXd start_inverse = T_bl_to_0th.inverse();
        // std::cout << "Homogeneous Matrix start inverse:\n" << start_inverse << "\n" << std::endl;

        // Hom_matrix = start_inverse * Hom_matrix;
        Hom_matrix = Hom_matrix * end_inverse;
        // std::cout << "Homogeneous Matrix after pre and post multiplying by inverse:\n" << Hom_matrix << "\n" << std::endl;


        //convert Eigen::Matrix4d to array of doubles in row-major ordering
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(T, Hom_matrix.rows(), Hom_matrix.cols()) = Hom_matrix;
        ROS_INFO("T: \n%f, %f, %f, %f,\n%f, %f, %f, %f,\n%f, %f, %f, %f,\n%f, %f, %f, %f\n\n", 
                    T[0], T[1], T[2], T[3], T[4], T[5], T[6], T[7], T[8], T[9], T[10], T[11], T[12], T[13], T[14], T[15]);


        // double q_act[6] = {-2.1424, -1.71789, -1.2125, 4.5006, 1.56903, -1.3569};
        // // double q_act[6] = {-2.5, -1.71789, -1.2125, 4.5006, 1.56903, -1.3569};

        // for (int i = 0; i < 6; i++){
        //     if (q_act[i] < 0)
        //         q_act[i] = q_act[i] + 2*M_PI; 
        //     std::cout << "q_act[" << i << "]: " << q_act[i] << std::endl;
        // }
        // double* T__ = new double[16];

        // kinematics.forward(q_act, T__);   
        // ROS_INFO("T__ after forward: \n%f, %f, %f, %f,\n%f, %f, %f, %f,\n%f, %f, %f, %f,\n%f, %f, %f, %f\n\n", 
        //             T__[0], T__[1], T__[2], T__[3], T__[4], T__[5], T__[6], T__[7], T__[8], T__[9], T__[10], T__[11], T__[12], T__[13], T__[14], T__[15]);


        //------COMPUTE INVERSE KINEMATICS------//

        //need to feed to the inverse kinematics the pose of the tcp/wrist 3 link wrt base_link (as a 4x4 homogeneous matrix)
        /*num_sols = kinematics.inverse(T, q_sols);
        ROS_INFO("num_sols: %d\n\n", num_sols);

        //define joint_variation between current joint values and the ones found with inverse kinematics
        Eigen::MatrixXd joint_variation(num_sols,6);

        //----------COMMAND JOINTS TO MOVE TO THE GOAL POSITION----------//

        if (num_sols==0){
            ROS_ERROR("Inverse kinematics not found\n\n");
        }
        else{
            ROS_INFO("Inverse kinematics found");
            std::cout<<"Current joint values: "<<current_joint_values[0]<<", "<<current_joint_values[1]<<", "<<current_joint_values[2]<<", "<<current_joint_values[3]<<", "<<current_joint_values[4]<<", "<<current_joint_values[5]<<std::endl;

            for(int i=0;i<num_sols;i++) 
                printf("Solution %d: %1.6f, %1.6f, %1.6f, %1.6f, %1.6f, %1.6f\n", i, q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]);
            

            //select the solution closest to the current joint values
            //convert current_joint_values to positive values, since the inverse kinematics returns only positive values and the robot can move in both directions
            for (int i = 0; i < 6; ++i){
                if (current_joint_values[i] < 0)
                    pos_joint_values[i] = current_joint_values[i] + 2*M_PI;
                else
                    pos_joint_values[i] = current_joint_values[i];
                std::cout << "pos_joint_values[" << i << "]: " << pos_joint_values[i] << std::endl;

                //compute joint variation between current_joint_values and q_sols
                for (int j = 0; j < num_sols; ++j){
                    joint_variation(j,i) = pos_joint_values[i]-q_sols[j*6+i];
                }
            }
            std::cout<<"\njoint_variation: \n"<<joint_variation<< "\n" << std::endl;

            //select the solution with the smallest norm
            sol_selected = 0;
            std::cout<<"norm 0: " <<joint_variation.row(0).norm()<< "\n" << std::endl;
            for (int j = 1; j < num_sols; ++j){
                
                if (joint_variation.row(j).norm() < joint_variation.row(sol_selected).norm())
                    sol_selected = j;

                std::cout<<"norm "<< j << ": " <<joint_variation.row(j).norm()<< "\n" << std::endl;
                
            }
            std::cout<<"Solution selected: "<< sol_selected << "\n" << std::endl;

            for (int i = 0; i < 6; ++i){
                joint_goal[i] = q_sols[sol_selected*6+i];
                // joint_goal[i] = current_joint_values[i];
                ROS_INFO("Joint %d: %f\n", i, joint_goal[i]);
            }
        }

        //command new joint values to the robot, using a trajectory message
        //to avoid undesired movements, the sign of the joint variation is taken into account (the value is equivalent)
        for (int i=0; i<6; i++){
            if (current_joint_values[i] < 0){
                traj.points[0].positions[i] = joint_goal[i] - 2*M_PI;
                // traj.points[0].velocities[i] = joint_vel_goal[i];
                // traj.points[0].accelerations[i] = joint_acc_goal[i];
            }
            else{
                traj.points[0].positions[i] = joint_goal[i];
                // traj.points[0].velocities[i] = joint_vel_goal[i];
                // traj.points[0].accelerations[i] = joint_acc_goal[i];
            }
            std::cout << "Trajectory coordinate "<< i <<": " <<traj.points[0].positions[i] << std::endl;
        }
        std::cout << "\n" << std::endl;

        
        traj.points[0].time_from_start = ros::Duration(0.5 + traj.points[0].time_from_start.toSec());*/
        pub.publish(traj);
        rate.sleep();
    }

    std::cout<<"\n\n\n\n"<<std::endl;
    ros::shutdown();
    return 0;
}

