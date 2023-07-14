#ifndef UR_KIN
#define UR_KIN

#include <ros/ros.h>
#include <iostream>
#include <ignition/math/Pose3.hh>
#include <cmath>
#include <string>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// namespace ur_kinematics {
  
  class Kinematics {

    public:

      enum ROBOT_TYPE {UR5, UR5e, UR10, UR10e, UR3};

      Kinematics();
      Kinematics(const std::string& URDF_name);
      Kinematics(Kinematics::ROBOT_TYPE);

      /*! \brief  Transform a pose from source frame to target frame
      \returns the transformed pose*/
      std::vector<double> tfPose_rpy(std::string sourceFrame, std::string targetFrame);
      std::vector<double> tfPose_quat(std::string sourceFrame, std::string targetFrame);


      Eigen::Matrix4d Vector_To_HomogeneusMatrix(double x, double y, double z, double roll, double pitch, double yaw);


      // @param q       The 6 joint values 
      // @param T       The 4x4 end effector pose in row-major ordering
      void forward(const double* q, double* T);

      // @param q       The 6 joint values 
      // @param Ti      The 4x4 link i pose in row-major ordering. If NULL, nothing is stored.
      void forward_all(const double* q, double* T1, double* T2, double* T3, 
                                        double* T4, double* T5, double* T6);

      // @param T       The 4x4 end effector pose in row-major ordering
      // @param q_sols  An 8x6 array of doubles returned, all angles should be in [0,2*PI)
      //                Choose one row as the joint positions.
      // @param q6_des  An optional parameter which designates what the q6 value should take
      //                in case of an infinite solution on that joint.
      // @return        Number of solutions found (maximum of 8)
      int inverse(const double* T, double* q_sols, double q6_des=0.0);

    private:

      double d1;
      double a2;
      double a3;
      double d4;
      double d5;
      double d6;  

      tf::TransformListener listener;
      
  };
// }

#endif // UR_KIN