#ifndef PID_CONTROLLER
#define PID_CONTROLLER

#include <ros/ros.h>
#include <iostream>
#include <ignition/math/Pose3.hh>
#include <cmath>
#include <string>
#include <vector>
#include <geometry_msgs/Wrench.h>
#include "geometry_msgs/WrenchStamped.h"

class PIDController {
public:
  PIDController(double kp_f, double ki_f, double kd_f, double kp_m, double ki_m, double kd_m)
      : kp_f_(kp_f), ki_f_(ki_f), kd_f_(kd_f), kp_m_(kp_m), ki_m_(ki_m), kd_m_(kd_m),
        last_error_f_(geometry_msgs::Wrench()), last_error_m_(geometry_msgs::Wrench()),
        integral_f_(geometry_msgs::Wrench()), integral_m_(geometry_msgs::Wrench()) {}

  geometry_msgs::Wrench compute(geometry_msgs::Wrench error, double dt) {
    geometry_msgs::Wrench control_signal;

    // Force components
    double derivative_fx = (error.force.x - last_error_f_.force.x) / dt;
    double derivative_fy = (error.force.y - last_error_f_.force.y) / dt;
    double derivative_fz = (error.force.z - last_error_f_.force.z) / dt;

    integral_f_.force.x += error.force.x * dt;
    integral_f_.force.y += error.force.y * dt;
    integral_f_.force.z += error.force.z * dt;

    control_signal.force.x = kp_f_ * error.force.x + ki_f_ * integral_f_.force.x + kd_f_ * derivative_fx;
    control_signal.force.y = kp_f_ * error.force.y + ki_f_ * integral_f_.force.y + kd_f_ * derivative_fy;
    control_signal.force.z = kp_f_ * error.force.z + ki_f_ * integral_f_.force.z + kd_f_ * derivative_fz;

    last_error_f_.force.x = error.force.x;
    last_error_f_.force.y = error.force.y;
    last_error_f_.force.z = error.force.z;

    // Torque components
    double derivative_mx = (error.torque.x - last_error_m_.torque.x) / dt;
    double derivative_my = (error.torque.y - last_error_m_.torque.y) / dt;
    double derivative_mz = (error.torque.z - last_error_m_.torque.z) / dt;

    integral_m_.torque.x += error.torque.x * dt;
    integral_m_.torque.y += error.torque.y * dt;
    integral_m_.torque.z += error.torque.z * dt;

    control_signal.torque.x = kp_m_ * error.torque.x + ki_m_ * integral_m_.torque.x + kd_m_ * derivative_mx;
    control_signal.torque.y = kp_m_ * error.torque.y + ki_m_ * integral_m_.torque.y + kd_m_ * derivative_my;
    control_signal.torque.z = kp_m_ * error.torque.z + ki_m_ * integral_m_.torque.z + kd_m_ * derivative_mz;

    last_error_m_.torque.x = error.torque.x;
    last_error_m_.torque.y = error.torque.y;
    last_error_m_.torque.z = error.torque.z;

    return control_signal;
  }

private:
  double kp_f_;                      // Proportional gain for force components
  double ki_f_;                      // Integral gain for force components
  double kd_f_;                      // Derivative gain for force components
  double kp_m_;                      // Proportional gain for torque components
  double ki_m_;                      // Integral gain for torque components
  double kd_m_;                      // Derivative gain for torque components
  
  geometry_msgs::Wrench last_error_f_;     // Last error value for force components
  geometry_msgs::Wrench last_error_m_;     // Last error value for torque components
  
  geometry_msgs::Wrench integral_f_;       // Integral term accumulation for force components
  geometry_msgs::Wrench integral_m_;       // Integral term accumulation for torque components
};

#endif