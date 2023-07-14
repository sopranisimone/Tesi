#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "sensor_msgs/JointState.h"

#include <iostream>

class TrajectoryPublisher
{
public:
    TrajectoryPublisher()
    {
        desired_positions.resize(6);
        desired_positions[0] = -1.57;
        desired_positions[1] = -0.76;
        desired_positions[2] = -1.76;
        desired_positions[3] = 3.913;
        desired_positions[4] = 1.563;
        desired_positions[5] = 0.0;

        // init_velocities.resize(6);
        // init_effort.resize(6);
    }
    void setDesiredPositions(std::vector<double> init_positions){this->desired_positions = init_positions;}
    // void setInitVelocities(std::vector<double> init_velocities){this->init_velocities = init_velocities;}
    // void setInitEffort(std::vector<double> init_effort){this->init_effort = init_effort;}
    void setReady(bool ready){this->ready = ready;}


    std::vector<double> getDesiredPositions(){return desired_positions;}
    // std::vector<double> getInitVelocities(){return init_velocities;}
    // std::vector<double> getInitEffort(){return init_effort;}
    bool getReady(){return ready;}


    void CommandCallback(const sensor_msgs::JointState& msg)
    {
        setDesiredPositions(msg.position);
        setReady(true);
    }

private:

    std::vector<double> desired_positions;
    bool ready;

};

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "slave_controller");
    ros::NodeHandle nh;

    TrajectoryPublisher trajectoryPublisher;

    ros::AsyncSpinner spinner(3); 
    spinner.start();

    //spins two threads, one for the subscriber and one for the publisher
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ur5e_arm_controller/command", 100);
    ros::Subscriber sub = nh.subscribe("/new_command", 100, &TrajectoryPublisher::CommandCallback, &trajectoryPublisher);


    std::vector<double> error(6);
    // int num_of_trajpoints = 6;
    // double cycletime = 6;
    // ros::Rate rate(1/cycletime);

    ros::Rate rate(500);

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

    for (int i=0; i<6; i++){
        ROS_INFO("Joint %d: %s\n", i, traj.joint_names[i].c_str()); 
    }

    traj.points.resize(1);
    traj.points[0].positions.resize(6);
    traj.points[0].positions[0] = -1.57;
    traj.points[0].positions[1] = -0.76;
    traj.points[0].positions[2] = -1.76;
    traj.points[0].positions[3] = 3.913;
    traj.points[0].positions[4] = 1.563;
    traj.points[0].positions[5] = 0.0;


    while(ros::ok()){ 
        
        for(int i=0; i<6; i++){
            error[i] = trajectoryPublisher.getDesiredPositions()[i] - traj.points[0].positions[i];
            traj.points[0].positions[i] += error[i];
        }
        
        traj.points[0].time_from_start = ros::Duration(0.1);
        ROS_INFO("Publishing trajectory\n");
        pub.publish(traj);
        rate.sleep();
    }

    
    ros::shutdown();
    return 0;
}

//$ rostopic pub -r 500 /new_command sensor_msgs/JointState  '{position:  [ -1.57, -0.76, -1.76, 3.913, 1.563, 0.5 ]}'