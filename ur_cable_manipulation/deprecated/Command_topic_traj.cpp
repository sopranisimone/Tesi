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
        init_positions.resize(6);
        init_velocities.resize(6);
        init_effort.resize(6);
    }
    void setInitPositions(std::vector<double> init_positions){this->init_positions = init_positions;}
    void setInitVelocities(std::vector<double> init_velocities){this->init_velocities = init_velocities;}
    void setInitEffort(std::vector<double> init_effort){this->init_effort = init_effort;}
    void setReady(bool ready){this->ready = ready;}


    std::vector<double> getInitPositions(){return init_positions;}
    std::vector<double> getInitVelocities(){return init_velocities;}
    std::vector<double> getInitEffort(){return init_effort;}
    bool getReady(){return ready;}


    void jointStateCallback(const sensor_msgs::JointState& msg)
    {
        std::vector<double>::const_iterator first = msg.position.begin();
        std::vector<double>::const_iterator last = msg.position.begin() + 8;
        std::vector<double> pos(first, last);

        pos[0] = pos[4];
        pos[2] = pos[1];
        pos[1] = pos[3];
        pos[3] = pos[5];
        pos[4] = pos[6];
        pos[5] = pos[7];
        // for (int i=0; i<6; i++){
        //     ROS_INFO("Position %d: %f", i, pos[i]); 
        // }
        pos.resize(6);

        first = msg.effort.begin();
        last = msg.effort.begin() + 8;
        std::vector<double> eff(first, last);

        eff[0] = eff[4];
        eff[2] = eff[1];
        eff[1] = eff[3];
        eff[3] = eff[5];
        eff[4] = eff[6];
        eff[5] = eff[7];
        eff.resize(6);
        
        setInitPositions(pos);
        setInitEffort(eff);
        setReady(true);
        // init_positions[0] = msg.position[4];
        // init_positions[1] = msg.position[3];
        // init_positions[2] = msg.position[1];
        // init_positions[3] = msg.position[5];
        // init_positions[4] = msg.position[6];
        // init_positions[5] = msg.position[7];
    }

private:

    std::vector<double> init_positions;
    std::vector<double> init_velocities;
    std::vector<double> init_effort;
    bool ready;

};

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "joint_trajectory_publisher");
    ros::NodeHandle nh;

    TrajectoryPublisher trajectoryPublisher;

    ros::AsyncSpinner spinner(3); 
    spinner.start();

    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ur5_arm_controller/command", 100);
    // ros::Subscriber sub = nh.subscribe("/joint_states", 100, &TrajectoryPublisher::jointStateCallback, &trajectoryPublisher);
    ROS_INFO("Waiting for /joint_states topic...\n");

    // Wait for the first set of joint states to arrive
    // ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh);
    ROS_INFO("Ready to send joint commands\n");

    // Get the initial joint states
    // std::vector<double> init_positions = trajectoryPublisher.getInitPositions();

    // for (int i = 0; i < 6; ++i)
    //     ROS_INFO("Position %d: %f\n", i, init_positions[i]); 

    int num_of_trajpoints = 6;
    double cycletime = 6;
    ros::Rate rate(1/cycletime);

    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    // ROS_INFO("Time: %f\n", traj.header.stamp.toSec());
    traj.header.frame_id = "ur5_base_link";

    traj.joint_names.resize(6);

    traj.joint_names[0] = "ur5_shoulder_pan_joint";
    traj.joint_names[1] = "ur5_shoulder_lift_joint";
    traj.joint_names[2] = "ur5_elbow_joint";
    traj.joint_names[3] = "ur5_wrist_1_joint";
    traj.joint_names[4] = "ur5_wrist_2_joint";
    traj.joint_names[5] = "ur5_wrist_3_joint";

    for (int i=0; i<6; i++){
        ROS_INFO("Joint %d: %s\n", i, traj.joint_names[i].c_str()); 
    }

    traj.points.resize(num_of_trajpoints);
    traj.points[0].positions.resize(6);
    traj.points[0].positions[0] = -1.57;
    traj.points[0].positions[1] = -0.76;
    traj.points[0].positions[2] = -1.76;
    traj.points[0].positions[3] = 3.913;
    traj.points[0].positions[4] = 1.563;
    traj.points[0].positions[5] = 0.0;

    for(int i=0; i<num_of_trajpoints;i++){

        traj.points[i].positions.resize(6);
        traj.points[i].positions[0] = traj.points[0].positions[0];
        traj.points[i].positions[1] = traj.points[0].positions[1];
        traj.points[i].positions[2] = traj.points[0].positions[2];
        traj.points[i].positions[3] = traj.points[0].positions[3];
        traj.points[i].positions[4] = traj.points[0].positions[4];
        traj.points[i].positions[5] = traj.points[0].positions[5] + i*0.2;
        ROS_INFO("JOINT 5 Position: %f\n", traj.points[i].positions[5]);

        // traj.points[i].velocities.resize(6);
        // traj.points[i].velocities[0] = 0;
        // traj.points[i].velocities[1] = 0;
        // traj.points[i].velocities[2] = 0;
        // traj.points[i].velocities[3] = 0;
        // traj.points[i].velocities[4] = 0;
        // traj.points[i].velocities[5] = 0;
        traj.points[i].time_from_start = ros::Duration(0.2 + i*cycletime/num_of_trajpoints);
        // traj.points[i].time_from_start = ros::Duration(1.0 + i*2);
        ROS_INFO("Time: %f\n", traj.points[i].time_from_start.toSec());

    }
    

    while(ros::ok()){ 
        
        ROS_INFO("Publishing trajectory\n");
        pub.publish(traj);
        // for (int i=0; i<6;i++)
        //     ROS_INFO("%d joint position: %f\n", i, trajectoryPublisher.getInitPositions()[i]);
        rate.sleep();

    }

    ros::shutdown();
    return 0;
}
