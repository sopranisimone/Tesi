#include "Utils.hpp"

using namespace Utils;

JointState::JointState() {
    joint_values.resize(6);
    joint_names.resize(6);

    joint_names = {"ur5e_shoulder_pan_joint", "ur5e_shoulder_lift_joint", "ur5e_elbow_joint", "ur5e_wrist_1_joint", "ur5e_wrist_2_joint", "ur5e_wrist_3_joint"};
}

JointState::JointState(ROBOT_TYPE robot_type){
    joint_values.resize(6);
    joint_names.resize(6);
    
    switch (robot_type){

        case UR5:
            joint_names = {"ur5_shoulder_pan_joint", "ur5_shoulder_lift_joint", "ur5_elbow_joint", "ur5_wrist_1_joint", "ur5_wrist_2_joint", "ur5_wrist_3_joint"};
            break;
        case UR5e:
            joint_names = {"ur5e_shoulder_pan_joint", "ur5e_shoulder_lift_joint", "ur5e_elbow_joint", "ur5e_wrist_1_joint", "ur5e_wrist_2_joint", "ur5e_wrist_3_joint"};
            break;
        case UR10:
            joint_names = {"ur10_shoulder_pan_joint", "ur10_shoulder_lift_joint", "ur10_elbow_joint", "ur10_wrist_1_joint", "ur10_wrist_2_joint", "ur10_wrist_3_joint"};
            break;
        case UR10e:
            joint_names = {"ur10e_shoulder_pan_joint", "ur10e_shoulder_lift_joint", "ur10e_elbow_joint", "ur10e_wrist_1_joint", "ur10e_wrist_2_joint", "ur10e_wrist_3_joint"};
            break;
        case UR3:
            joint_names = {"ur3_shoulder_pan_joint", "ur3_shoulder_lift_joint", "ur3_elbow_joint", "ur3_wrist_1_joint", "ur3_wrist_2_joint", "ur3_wrist_3_joint"};
            break;
        default:
            ROS_ERROR("Robot type not defined");
            break;

    }
}

void JointState::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {

    lock = true;
    joint_values.clear();

    // Iterate over the desired joint names
    for (const std::string& jointName : joint_names){

        // Find the index of the current joint name in the JointState message
        auto it = std::find(msg->name.begin(), msg->name.end(), jointName);

        // Check if the joint name was found in the JointState message
        if (it != msg->name.end()){
            // Calculate the index based on the iterator position
            int index = std::distance(msg->name.begin(), it);

            // Access the joint value using the index
            double jointValue = msg->position[index];

            // Add the joint value to the joint_values vector
            joint_values.push_back(jointValue);
        }
        else{
            ROS_WARN_STREAM("Joint '" << jointName << "' not found in JointState message");
        }
    }

    // Check if all desired joint values were found
    if (joint_values.size() != joint_names.size()){
        ROS_WARN("Unable to retrieve all desired joint values from JointState message");
    }
    lock = false;
}

std::vector<double> JointState::getJointState(){ 
    while (lock == true) {
        ros::spinOnce();
    }
    return joint_values; 
};