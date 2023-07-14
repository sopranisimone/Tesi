#include "AdmittanceControl.hpp"

AdmittanceController::AdmittanceController(ros::Rate rate_) : M_d(6,6), D_d(6,6), K_d(6,6), loop_rate_(rate_)
{
    M_d <<  1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;

    D_d <<  10, 0, 0, 0, 0, 0,
            0, 10, 0, 0, 0, 0,
            0, 0, 10, 0, 0, 0,
            0, 0, 0, 10, 0, 0,
            0, 0, 0, 0, 10, 0,
            0, 0, 0, 0, 0, 10;

    K_d <<  10, 0, 0, 0, 0, 0,
            0, 10, 0, 0, 0, 0,
            0, 0, 10, 0, 0, 0,
            0, 0, 0, 5, 0, 0,
            0, 0, 0, 0, 5, 0,
            0, 0, 0, 0, 0, 5;

    // std::ofstream outputFile("/home/software/cable_manipulation_ws/src/ur_cable_manipulation/src/Memory.txt");

    // outputFile.close();
}

Eigen::VectorXd AdmittanceController::computeAdmittance(std::vector<double> force, std::vector<double> torque, std::vector<double> reference_pose)
{

    // std::ofstream outputFile("/home/software/cable_manipulation_ws/src/ur_cable_manipulation/src/Memory.txt", std::ios::app);

    // if (outputFile.is_open()) {

        ros::Duration dt = loop_rate_.expectedCycleTime();
        std::cout << "dt: " << dt.toSec() << std::endl;
        Eigen::VectorXd F_ext(6);
        F_ext << force[0], force[1], force[2], torque[0], torque[1], torque[2];
        // F_ext << 0, force[1], 0, 0, 0, 0;


        //DEBUG
        // F_ext << 0, 2, 0, 0, 0, 0;
        // for (int i = 0; i < F_ext.size(); i++)
        // {
        //     if (abs(F_ext(i)) < 10)
        //         F_ext(i) = 0;
        // }

        for (int i=0; i<reference_pose.size(); i++){
            pos(i) = reference_pose[i];
        }
        
        //delta_acc = qddot - qddot_d
        delta_acc = M_d.inverse() * (F_ext - D_d * delta_vel - K_d * delta_pos);
        // ROS_INFO_STREAM("delta acc: \n" << delta_acc);

        delta_vel += delta_acc * dt.toSec();
        // ROS_INFO_STREAM("delta vel: \n" << delta_vel);

        delta_pos += delta_vel * dt.toSec();
        // ROS_INFO_STREAM("delta pos: \n" << delta_pos);

        for (int i=0; i<reference_pose.size(); i++){
            pos(i) += delta_pos(i);
            // if (abs(delta_pos(i)) < 0.1) 
            //     delta_pos(i) = 0;
            // ROS_INFO_STREAM("delta pos[" << i << "]: " << delta_pos(i));
        }
        std::cout << "\n\n"<< std::endl;

        Eigen::VectorXd pose = pos.segment(0, 3);
        if (pose.norm() > 0.85){
            ROS_ERROR_STREAM("Pose out of range: "<< pose << std::endl);
            ROS_ERROR_STREAM("Pose out of range: "<< pose.norm() << std::endl);
            // outputFile << "POSE OUT OF RANGE: "<< std::endl;
            throw Exception("POSE OUT OF RANGE.");
        }

        // for (int i=0; i<delta_pos.size(); i++){
        //     if (delta_pos(i) != 0 && i!=1)
        //         throw Exception("DELTA POS NOT ZERO.");
        // }

        //DEBUG
        // outputFile << pos << std::endl;
        // for (int i = 0; i < pos.size(); ++i) {
        //     outputFile << pos[i] << " "; // Use a space as a delimiter
        // }
        // outputFile << std::endl;

        // outputFile.close();
        // std::cout << "File written successfully." << std::endl;
    // } else {
        // ROS_ERROR("Failed to open the file.\n\n\n\n\n\n");
    // }

    return pos;
}





























// Eigen::VectorXd AdmittanceController::computeAdmittance(std::vector<double> force, std::vector<double> torque, std::vector<double> current_joint_values)
// {
//     ros::Duration dt = loop_rate_.expectedCycleTime();
//     std::cout << "dt: " << dt.toSec() << std::endl;
//     Eigen::VectorXd F_ext(6);
//     F_ext << force[0], force[1], force[2], torque[0], torque[1], torque[2];

//     for (int i=0; i<F_ext.size(); i++){
//         ROS_INFO_STREAM("F_ext[" << i << "]: " << F_ext(i));
//         // if (std::abs(F_ext(i)) < 35){
//         //     F_ext(i) = 0;
//         // }
//         //else  F_ext(i) *= 10;
//     }

//     for (int i=0; i<current_joint_values.size(); i++){
//         pos(i) = current_joint_values[i];
//     }
    
//     //delta_acc = qddot - qddot_d
//     delta_acc = M_d.inverse() * (F_ext - D_d * delta_vel - K_d * delta_pos);
//     // ROS_INFO_STREAM("delta acc: " << delta_acc);

//     delta_vel += delta_acc * dt.toSec();
//     // ROS_INFO_STREAM("delta vel: " << delta_vel);

//     delta_pos += delta_vel * dt.toSec();
//     // ROS_INFO_STREAM("delta pos: " << delta_pos);

//     pos += delta_pos;

//     // for (int i=0; i<current_joint_values.size(); i++){
//     //     ROS_INFO_STREAM("delta pos[" << i << "]: " << delta_pos(i));
//     // }
//     std::cout << "\n\n"<< std::endl;

//     return pos;
// }
