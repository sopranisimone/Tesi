#ifndef ADMITTANCECONTROLLER_H
#define ADMITTANCECONTROLLER_H

#include "ros/ros.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#include <fstream>
#include <cstring>
#include <iostream>

#include <exception>
#include <string>

class Exception : public std::exception {
private:
    std::string errorMessage;

public:
    Exception(const std::string& message) : errorMessage(message) {}

    // Override the what() function to provide an error message
    const char* what() const noexcept override {
        return errorMessage.c_str();
    }
};

class AdmittanceController
{
public:
    AdmittanceController(ros::Rate rate_);

    Eigen::VectorXd computeAdmittance(std::vector<double> force, std::vector<double> torque, std::vector<double> reference_pose);

private:

    Eigen::MatrixXd M_d = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd D_d = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd K_d = Eigen::MatrixXd::Zero(6,6);

    ros::Rate loop_rate_;

    Eigen::VectorXd pos = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd delta_acc = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd delta_vel = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd delta_pos = Eigen::VectorXd::Zero(6);

};

#endif // ADMITTANCECONTROLLER_H