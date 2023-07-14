#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "sensor_msgs/JointState.h"

class TrajectoryPublisher
{
public:
    TrajectoryPublisher()
    {
        init_positions.resize(6);
        init_velocities.resize(6);
        init_effort.resize(6);
    }

    TrajectoryPublisher(trajectory_msgs::JointTrajectory traj_msg)
    {
        init_positions.resize(6);
        init_velocities.resize(6);
        init_effort.resize(6);

        traj_msg.joint_names.resize(6);

        // traj_msg.joint_names.push_back("ur5_shoulder_pan_joint");
        // traj_msg.joint_names.push_back("ur5_shoulder_lift_joint");
        // traj_msg.joint_names.push_back("ur5_elbow_joint");
        // traj_msg.joint_names.push_back("ur5_wrist_1_joint");
        // traj_msg.joint_names.push_back("ur5_wrist_2_joint");
        // traj_msg.joint_names.push_back("ur5_wrist_3_joint");

        traj_msg.joint_names[0]= "ur5_shoulder_pan_joint";
        traj_msg.joint_names[1]= "ur5_shoulder_lift_joint";
        traj_msg.joint_names[2]= "ur5_elbow_joint";
        traj_msg.joint_names[3]= "ur5_wrist_1_joint";
        traj_msg.joint_names[4]= "ur5_wrist_2_joint";
        traj_msg.joint_names[5]= "ur5_wrist_3_joint";

        traj_msg.header.frame_id = "ur5_base_link";


    }

    void setInitPositions(std::vector<double> init_positions)
    {
        this->init_positions = init_positions;
    }

    void setInitVelocities(std::vector<double> init_velocities)
    {
        this->init_velocities = init_velocities;
    }

    void setInitEffort(std::vector<double> init_effort)
    {
        this->init_effort = init_effort;
    }

    void setReady(bool ready)
    {
        this->ready = ready;
    }


    std::vector<double> getInitPositions()
    {
        return init_positions;
    }

    std::vector<double> getInitVelocities()
    {
        return init_velocities;
    }

    std::vector<double> getInitEffort()
    {
        return init_effort;
    }

    bool getReady()
    {
        return ready;
    }


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

        // setInitVelocities(msg.velocity);
        // setInitEffort(msg.effort);

        // std::cout<<"Joint State Callback\n"<< msg << std::endl;

    }

    bool isGoalReached(float goal){
        float current_position;
        // ROS_INFO("Goal: %f", goal);
        std::vector<double> pos = getInitPositions();
        for (int i=0; i<6; i++){
            // ROS_INFO("Position %d: %f", i, pos[i]);
            current_position += pos[i];
        }
        // ROS_INFO("Current position: %f", current_position);

        if (abs(goal - current_position) > 0.01){
            // ROS_INFO("Goal not reached: %f", abs(goal - current_position));
            return false;
        }
        // ROS_INFO("Goal reached");
        return true;
    }

    float computeGoal(std::vector<double> positions){
        float goal = 0.0;
        for (int i=0; i<6; i++){
            goal +=  positions[i];
        }
        
        return goal;
    }

    bool moveJoint(trajectory_msgs::JointTrajectory traj_msg, ros::Publisher pub, std::vector<double> positions, float duration){
        
        traj_msg.points[0].positions.resize(6);
        traj_msg.points[0].velocities.resize(6);
        traj_msg.points[0].accelerations.resize(6);
        traj_msg.points[0].effort.resize(6);
        traj_msg.points[0].time_from_start = ros::Duration(duration);

        traj_msg.points[0].positions = positions;
        
        ros::Rate loop_rate(10);
        float goal = this->computeGoal(positions);
        while (!this->isGoalReached(goal)){
            pub.publish(traj_msg);
        }

        return true;
    }


    bool setWaypoint(trajectory_msgs::JointTrajectory traj_msg, int number, std::vector<double> positions, std::vector<double> velocities, float duration){
        traj_msg.points[number].positions.resize(6);
        traj_msg.points[number].velocities.resize(6);
        traj_msg.points[number].accelerations.resize(6);
        traj_msg.points[number].effort.resize(6);
        traj_msg.points[number].time_from_start = ros::Duration(duration);
        traj_msg.points[number].positions = positions;
        traj_msg.points[number].velocities = velocities;
        return true;
    }

    bool executeTrajectory(trajectory_msgs::JointTrajectory traj_msg, ros::Publisher pub){
        ros::Rate loop_rate(10);
        while (ros::ok()){
            pub.publish(traj_msg);
            loop_rate.sleep(); 
        }

        return true;
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
    float goal = 0.0;

    ros::AsyncSpinner spinner(3); 
    spinner.start();

    // ros::waitForShutdown();

    // Create the publisher
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ur5_arm_controller/command", 100);

    ros::Subscriber sub = nh.subscribe("/joint_states", 100, &TrajectoryPublisher::jointStateCallback, &trajectoryPublisher);

    ros::Duration(8.0).sleep();
    // Set the loop rate
    // ros::Rate loop_rate(10);

    // Create the JointTrajectory message
    trajectory_msgs::JointTrajectory traj_msg;

    ROS_INFO("NAMES SIZE: %ld", traj_msg.joint_names.size());
    // traj_msg.points[0].positions.resize(8);
    // traj_msg.joint_names.push_back("ur5_shoulder_pan_joint");
    // traj_msg.joint_names.push_back("ur5_shoulder_lift_joint");
    // traj_msg.joint_names.push_back("ur5_elbow_joint");
    // traj_msg.joint_names.push_back("ur5_wrist_1_joint");
    // traj_msg.joint_names.push_back("ur5_wrist_2_joint");
    // traj_msg.joint_names.push_back("ur5_wrist_3_joint");
    // traj_msg.joint_names.push_back("ur5_bl_to_leftFinger");
    // traj_msg.joint_names.push_back("ur5_leftFinger_to_rightFinger");


    traj_msg.header.frame_id = "ur5_base_link";
    // traj_msg.header.frame_id = "world";

    std::vector<double> positions(6);
    std::vector<double> velocities(6);
    std::vector<double> accelerations(6);
    std::vector<double> effort(6);

    while (!trajectoryPublisher.getReady()) {
        ros::Duration(0.5).sleep();
        ROS_INFO("Waiting for joint state message");
    }
    positions = trajectoryPublisher.getInitPositions();
    positions[5] = 3.14; 
    

    traj_msg.header.stamp = ros::Time::now();
    traj_msg.points.resize(1);

    if (trajectoryPublisher.moveJoint(traj_msg, pub, positions, 1.0)){
        ROS_INFO("\n\n--------------------First goal reached-------------------------------------\n\n");
    }

    positions = trajectoryPublisher.getInitPositions();
    positions[4] = 0.0;

    if (trajectoryPublisher.moveJoint(traj_msg, pub, positions, 2.0)){
        ROS_INFO("\n\n--------------------Second goal reached-------------------------------------\n\n");
    }


    positions = trajectoryPublisher.getInitPositions();
    positions[5] = 0.0;

    traj_msg.header.stamp = ros::Time::now();
    traj_msg.points.resize(4);

    trajectoryPublisher.setWaypoint(traj_msg, 0, positions, velocities, 10.0);

    positions[5] = 1.57;
    velocities[5] = 0.5;
    trajectoryPublisher.setWaypoint(traj_msg, 1, positions, velocities, 20.0);

    positions[5] = 3.14;
    velocities[5] = 0.5;
    trajectoryPublisher.setWaypoint(traj_msg, 2, positions, velocities, 30.0);

    positions[5] = 0.0;
    velocities[5] = 0.0;
    trajectoryPublisher.setWaypoint(traj_msg, 3, positions, velocities, 40.0);

    trajectoryPublisher.executeTrajectory(traj_msg, pub);

    

    // while (ros::ok)
    // {
    //     //ROS_INFO("\n\nStarting publishing trajectory message\n\n");

    //     // ros::Duration(6.0).sleep();
    //     ros::Duration(2.0).sleep();
    //     traj_msg.header.stamp = ros::Time::now();
        

    //     // trajectory_msgs::JointTrajectoryPoint point;
    //     // point.positions.push_back(1.0);
    //     // point.positions.push_back(2.0);
    //     // point.positions.push_back(3.0);
    //     // point.time_from_start = ros::Duration(1.0);

    //     std::vector<double> positions(6);
    //     // positions[0] = -1.54;
    //     // positions[1] = -0.76;
    //     // positions[2] = -1.76;
    //     // positions[3] = 3.92;
    //     // positions[4] = 1.55;
    //     // positions[5] = 3.14;

    //     // positions[0] = trajectoryPublisher.init_positions[0];
    //     // positions[1] = trajectoryPublisher.init_positions[1];
    //     // positions[2] = trajectoryPublisher.init_positions[2];
    //     // positions[3] = trajectoryPublisher.init_positions[3];
    //     // positions[4] = trajectoryPublisher.init_positions[4];
    //     // positions[5] = trajectoryPublisher.init_positions[5] + 3.14;

    //     positions = trajectoryPublisher.getInitPositions();
    //     positions[5] = 3.14; 
    //     // positions.resize(6);
        
    //     // for (int i = 0; i<6; i++){
    //     //     ROS_INFO("Position %d: %f", i, positions[i]);
    //     //     goal += goal + positions[i];
    //     // }
    //     goal = trajectoryPublisher.computeGoal(positions);

    //     std::vector<double> velocities(6);
    //     velocities[0] = 0.0;
    //     velocities[1] = 0.0;
    //     velocities[2] = 0.0;
    //     velocities[3] = 0.0;
    //     velocities[4] = 0.0;
    //     velocities[5] = 0.0;
        
    //     std::vector<double> accelerations(6);
    //     // accelerations[0] = 0.4;
    //     // accelerations[1] = 0.4;
    //     // accelerations[2] = 0.4;
    //     // accelerations[3] = 0.4;
    //     // accelerations[4] = 0.4;
    //     // accelerations[5] = 0.4;
    //     // accelerations[6] = 0.4;
    //     // accelerations[7] = 0.4;

    //     std::vector<double> effort(6);
    //     // effort[0] = 0.4;
    //     // effort[1] = 0.4;
    //     // effort[2] = 0.4;
    //     // effort[3] = 0.4;
    //     // effort[4] = 0.4;
    //     // effort[5] = 0.4;
    //     // effort[6] = 0.4;

    //     // Create a new trajectory point and add it to the trajectory message
    //     traj_msg.points.resize(1);
    //     traj_msg.points[0].positions = positions;
    //     traj_msg.points[0].velocities = velocities;
    //     traj_msg.points[0].accelerations = accelerations;
    //     traj_msg.points[0].effort = effort;
    //     traj_msg.points[0].time_from_start = ros::Duration(1.0);

        
    //     while (!trajectoryPublisher.isGoalReached(goal)){
    //         // goal = trajectoryPublisher.computeGoal(positions);
    //         pub.publish(traj_msg);
    //     }
    //     // pub.publish(traj_msg);

    //     // positions[0] += 3.14;
    //     // positions[1] += 3.14;
    //     // positions[4] += 1.57;

    //     // traj_msg.points[1].positions = positions;
    //     // traj_msg.points[1].velocities = velocities;
    //     // traj_msg.points[1].accelerations = accelerations;
    //     // traj_msg.points[1].effort = effort;
    //     // traj_msg.points[1].time_from_start = ros::Duration(3.0);

    //     // goal = trajectoryPublisher.computeGoal(trajectoryPublisher.getInitPositions());
    //     // while (!trajectoryPublisher.isGoalReached(goal)){
    //     //     // goal = trajectoryPublisher.computeGoal(positions);
    //     //     pub.publish(traj_msg);
    //     // }

    //     // traj_msg.points.push_back(point);

    //     // Publish the message
       

    //     ROS_INFO("Published trajectory message p[5] = %f, p[4] = %f\n\n", traj_msg.points[0].positions[5], traj_msg.points[0].positions[4]);

    //     // Spin once and sleep
    //     ros::spinOnce();
    //     loop_rate.sleep();

    // }
    
    ros::shutdown();
    return 0;
}
