#ifndef UTILS
#define UTILS

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <string>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "sensor_msgs/JointState.h"

namespace Utils {
    class JointState {
        public:
            enum ROBOT_TYPE {UR5, UR5e, UR10, UR10e, UR3};

            JointState();
            JointState(JointState::ROBOT_TYPE);
            void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
            std::vector<double> getJointState();

        private:
            std::vector<double> joint_values;
            std::vector<std::string> joint_names;
            bool lock = false;

    };
}

#endif // UTILS