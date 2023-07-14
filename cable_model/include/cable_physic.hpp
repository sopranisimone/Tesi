#ifndef CABLE_PHYSIC
#define CABLE_PHYSIC


#include <ignition/math/Pose3.hh>
#include "geometry_msgs/Pose.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "physic.hpp"
#include "cable.hpp"
#include "utils.hpp"
#include <numeric>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>
#include <string>
#include <fstream>

namespace cable_physic{
    
    class MassSpringDamping {
        protected:
            double mass, stiffness, damping, bending;
            ignition::math::Vector3d g;
            bool gravity = true;
            bool testing = false;
            bool wiring = false;
            bool start_grasp = false;
            bool stop_grasp = false;
            ignition::math::Pose3d end_fixed_pose;
            const ignition::math::Vector3d f_g = {0, 0, -9.81};
            void computeForces(std::shared_ptr<tiago_cable_manipulation::Cable> cable/*, double ts*/, int resolution);
            std::vector <ignition::math::Vector3d> computeBendingTwisting(std::shared_ptr<tiago_cable_manipulation::Cable> cable);
            ignition::math::Vector3d getVector(int i, int j, std::shared_ptr<tiago_cable_manipulation::Cable> cable);
            ignition::math::Vector3d getVector(int i, std::shared_ptr<tiago_cable_manipulation::Cable> cable);
            ignition::math::Vector3d getUnitVector(int i, std::shared_ptr<tiago_cable_manipulation::Cable> cable);
            ignition::math::Vector3d tripleCross(int i, int j, int k, std::shared_ptr<tiago_cable_manipulation::Cable> cable);
            ignition::math::Vector3d tripleCross(ignition::math::Vector3d u1,
                                                                ignition::math::Vector3d u2,
                                                                ignition::math::Vector3d u3,
                                                                std::shared_ptr<tiago_cable_manipulation::Cable> cable);
            double getBeta(int i, std::shared_ptr<tiago_cable_manipulation::Cable> cable);
            double getBeta(ignition::math::Vector3d link_vec, ignition::math::Vector3d fixed_vec, std::shared_ptr<tiago_cable_manipulation::Cable> cable);
            void setGraspedParticle(int i);
            int getGraspedParticle();
        public:
            const float MAX_FORCE = 100;
            int grasped_particle;
            MassSpringDamping(double mass, double stiffness, double damping, double bending, bool testing, bool wiring, bool simulateGravity = true, ignition::math::Vector3d gravity = {0, 0, -9.81});
            virtual void computePositions(std::shared_ptr<tiago_cable_manipulation::Cable> cable, bool testing, bool wiring);
            void virtualGrasp(int i, geometry_msgs::Pose ee_pose, std::shared_ptr<tiago_cable_manipulation::Cable> cable, bool stop_grasp);
            void fixEnds(std::shared_ptr<tiago_cable_manipulation::Cable> cable, bool cable_inserted);
            ros::NodeHandle nh;
    };
}
#endif