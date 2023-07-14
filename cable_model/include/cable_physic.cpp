#include "cable_physic.hpp"


MassSpringDamping::MassSpringDamping(double mass, double stiffness, double damping, double bending, bool testing, bool wiring, bool simulateGravity, ignition::math::Vector3d gravity) :
    mass(mass), stiffness(stiffness), damping(damping), bending(bending), gravity(simulateGravity), g(gravity), testing(testing), wiring(wiring) {}


ignition::math::Vector3d MassSpringDamping::getVector(int i, int j, std::shared_ptr<tiago_cable_manipulation::Cable> cable) { 
    return cable->getLink(i)->WorldPose().Pos() - cable->getLink(j)->WorldPose().Pos();
}

ignition::math::Vector3d MassSpringDamping::getVector(int i, std::shared_ptr<tiago_cable_manipulation::Cable> cable) { 
    return cable->getLink(i)->WorldPose().Pos() - cable->getLink(i-1)->WorldPose().Pos();
}

ignition::math::Vector3d MassSpringDamping::getUnitVector(int i, std::shared_ptr<tiago_cable_manipulation::Cable> cable) { 
    ignition::math::Vector3d vector = getVector(i, cable);
    return vector / vector.Length();
}

ignition::math::Vector3d MassSpringDamping::tripleCross(int i, int j, int k, std::shared_ptr<tiago_cable_manipulation::Cable> cable) { 
    ignition::math::Vector3d u1, u2, u3;
    u1 = getUnitVector(i, cable);
    u2 = getUnitVector(j, cable);
    u3 = getUnitVector(k, cable);
    return u1.Cross(u2.Cross(u3));
}

ignition::math::Vector3d MassSpringDamping::tripleCross(ignition::math::Vector3d u1,
                                                        ignition::math::Vector3d u2,
                                                        ignition::math::Vector3d u3,
                                                        std::shared_ptr<tiago_cable_manipulation::Cable> cable) { 
    return u1.Cross(u2.Cross(u3));
}


double MassSpringDamping::getBeta(int i, std::shared_ptr<tiago_cable_manipulation::Cable> cable) { 
  return atan((getVector(i+1, cable).Cross(getVector(i, cable))).Length() / getVector(i+1, cable).Dot(getVector(i, cable)));
}

double MassSpringDamping::getBeta(ignition::math::Vector3d link_vec, ignition::math::Vector3d fixed_vec, std::shared_ptr<tiago_cable_manipulation::Cable> cable) { 
  return atan((link_vec.Cross(fixed_vec)).Length() / link_vec.Dot(fixed_vec));
}



void MassSpringDamping::computePositions(std::shared_ptr<tiago_cable_manipulation::Cable> cable, bool testing, bool wiring) {

    int n = cable->getResolution();
    int grabbed_particle;
    std::vector <double> ee_pose_position;
    std::vector <double> ee_pose_orientation;
    geometry_msgs::Pose ee_pose;
    ignition::math::Pose3d ee_pose_ign;
    bool cable_inserted = false;


    if(testing) {
        cable->get(n-1).setFixed(true);
        //cable->get(n-1).setFixed(true);
    }
    if(wiring) {
        nh.getParam("/cable/cable_inserted", cable_inserted);
        //if(cable_inserted) cable->get(n-1).setFixed(true);
        //cable->get(0).setFixed(true);
        fixEnds(cable, cable_inserted);
    }
    computeForces(cable, n);

    if(!testing) {
        nh.getParam("cable/start_grasp", start_grasp);
        nh.getParam("cable/stop_grasp", stop_grasp);
        nh.getParam("cable/grabbed_particle", grabbed_particle);



        gazebo::physics::WorldPtr world;
        gazebo::physics::ModelPtr tiago;
        world = cable->getLink(0)->GetModel()->GetWorld();
        tiago = world->ModelByName("tiago");
        ee_pose_ign = tiago->GetLink("arm_7_link")->WorldPose();

        
        ee_pose.position.y = ee_pose_ign.Pos().Y();

        if(!wiring) {
            ee_pose.position.x = ee_pose_ign.Pos().X();
            ee_pose.position.z = ee_pose_ign.Pos().Z() - 0.283; //cause move_group.currentPose() is shifted with respect to arm_7_link
        }
        if(wiring) {
            ee_pose.position.x = ee_pose_ign.Pos().X() + 0.2713;
            ee_pose.position.z = ee_pose_ign.Pos().Z();
        }

        ee_pose.orientation.x = ee_pose_ign.Rot().X();
        ee_pose.orientation.y = ee_pose_ign.Rot().Y();
        ee_pose.orientation.z = ee_pose_ign.Rot().Z();
        ee_pose.orientation.w = ee_pose_ign.Rot().W();


        //if(start_grasp) virtualGrasp(grabbed_particle, ee_pose, cable, stop_grasp);
        
    }
}



void MassSpringDamping::computeForces(std::shared_ptr<tiago_cable_manipulation::Cable> cable/*, double ts*/, int resolution) {
    
    double n, n0;
    //double ts_p2 = ts * ts;
    std::vector <ignition::math::Vector3d> f, d, b;
    ignition::math::Vector3d linear_forces, damping_forces, bending_forces;
    ignition::math::Vector3d result;
    ignition::math::Vector3d l0, lt, l_prova;
    ignition::math::Quaternion<double> q(0.0, 0.0, 0.0);
    //q.setRPY(0.0, 0.0, 0.0);

    double beta;

    for(int i = 0; i < resolution; i++) {
        ignition::math::Pose3d p(cable->getLink(i)->WorldPose().Pos(), q);
        cable->getLink(i)->SetWorldPose(p); //fixing links frames, needed for a better visual if particles are not spheres
        if(cable->get(i).isFixed()) {
            f.clear();
            d.clear();
            b.clear();
            continue;
        }
        //========== LINEAR SPRING FORCES + DAMPING ===========//
        for(int neighbour = -1; neighbour <= 1; neighbour++) {
            if(neighbour + i < 0 || neighbour + i >= resolution || neighbour == 0) {
                continue;
            }
            lt = getVector(i, i + neighbour, cable);
            l0 = cable->getLink(i)->InitialRelativePose().Pos() - cable->getLink(i + neighbour)->InitialRelativePose().Pos();
            n0 = l0.Length();
            n = lt.Length();
            f.push_back(-stiffness * (n - n0) * (lt / n));
            d.push_back(-damping * (cable->getLink(i)->WorldLinearVel() - cable->getLink(i + neighbour)->WorldLinearVel()));
        }
        //========== BENDING SPRING FORCES ===========//


        //l_i = getVector(i, cable)
        //l_i+1 = getVector(i+1, cable)
        //beta_i+1 = getBeta(i+1, cable)
        //u_i = getUnitVector(i, cable)
        //u_i+1 = getUnitVector(i+1, cable)
        //u_i x (u_i-1 x u_i) = tripleCross(i, i-1, i, cable)

        ignition::math::Vector3d fixed_axis = {0.0, 1.0, 0.0};

        if(i == 0) {
            b.push_back(bending * getBeta(i+1, cable) / getVector(i+1, cable).Length() * tripleCross(i+1, i+1, i+2, cable) / sin(getBeta(i+1, cable)));
        }

        if(i == 1) {
            if(cable->get(0).isFixed()) {    
            b.push_back(bending * getBeta(getVector(i, cable), fixed_axis, cable) / getVector(i, cable).Length() * tripleCross(getUnitVector(i, cable), fixed_axis, getUnitVector(i, cable), cable) / sin(getBeta(getVector(i, cable), fixed_axis, cable)));
            }

            b.push_back(-bending * getBeta(i, cable) / getVector(i, cable).Length() * tripleCross(i, i, i+1, cable) / sin(getBeta(i, cable)));    
            b.push_back(-bending * getBeta(i, cable) / getVector(i+1, cable).Length() * tripleCross(i+1, i, i+1, cable) / sin(getBeta(i, cable)));
            b.push_back(bending * getBeta(i+1, cable) / getVector(i+1, cable).Length() * tripleCross(i+1, i+1, i+2, cable) / sin(getBeta(i+1, cable)));   
        }

        if(i == resolution - 2) {
            b.push_back(bending * getBeta(i-1, cable) / getVector(i, cable).Length() * tripleCross(i, i-1, i, cable) / sin(getBeta(i-1, cable)));    
            b.push_back(-bending * getBeta(i, cable) / getVector(i, cable).Length() * tripleCross(i, i, i+1, cable) / sin(getBeta(i, cable)));    
            b.push_back(-bending * getBeta(i, cable) / getVector(i+1, cable).Length() * tripleCross(i+1, i, i+1, cable) / sin(getBeta(i, cable)));
            
            if(cable->get(resolution - 1).isFixed()) { 
            b.push_back(bending * getBeta(-fixed_axis, getVector(i+1, cable), cable) / getVector(i+1, cable).Length() * tripleCross(getUnitVector(i+1, cable), getUnitVector(i+1, cable), -fixed_axis, cable) / sin(getBeta(-fixed_axis, getVector(i+1, cable), cable)));
            }
        }

        if(i == resolution - 1) {
            b.push_back(bending * getBeta(i-1, cable) / getVector(i, cable).Length() * tripleCross(i, i-1, i, cable) / sin(getBeta(i-1, cable)));    
        }

        if(i > 1 && i < resolution - 2 ) {
        b.push_back(bending * getBeta(i-1, cable) / getVector(i, cable).Length() * tripleCross(i, i-1, i, cable) / sin(getBeta(i-1, cable)));    
        b.push_back(-bending * getBeta(i, cable) / getVector(i, cable).Length() * tripleCross(i, i, i+1, cable) / sin(getBeta(i, cable)));    
        b.push_back(-bending * getBeta(i, cable) / getVector(i+1, cable).Length() * tripleCross(i+1, i, i+1, cable) / sin(getBeta(i, cable)));
        b.push_back(bending * getBeta(i+1, cable) / getVector(i+1, cable).Length() * tripleCross(i+1, i+1, i+2, cable) / sin(getBeta(i+1, cable)));    
        }
        
        linear_forces = std::accumulate(f.begin(), f.end(), ignition::math::Vector3d::Zero);
        damping_forces = std::accumulate(d.begin(), d.end(), ignition::math::Vector3d::Zero);
        bending_forces = std::accumulate(b.begin(), b.end(), ignition::math::Vector3d::Zero);
        bending_forces.Correct();

        result = linear_forces + damping_forces + bending_forces;
        if(!gravity){
            result -= mass * g;
        }


        cable->get(i).setForceCache(result);
        f.clear();
        d.clear();
        b.clear();
    }
    
}

void MassSpringDamping::setGraspedParticle(int i) {
    grasped_particle = i;
}

int MassSpringDamping::getGraspedParticle() {
    return grasped_particle;
}

void MassSpringDamping::virtualGrasp(int i, geometry_msgs::Pose ee_pose, std::shared_ptr<tiago_cable_manipulation::Cable> cable, bool stop_grasp) {

    ignition::math::Vector3d p_pos = {ee_pose.position.x, ee_pose.position.y, ee_pose.position.z};
    ignition::math::Pose3d pose;
    ignition::math::Quaternion<double> rot;

    rot.Euler(0.0, 0.0, 0.0);
    pose.Set(p_pos, rot);
    if(!stop_grasp) {
        
        cable->get(i).setFixed(true);
        cable->getLink(i)->SetWorldPose(pose);
    }
    if(stop_grasp){
        if(cable->get(i).isFixed()) cable->get(i).setFixed(false);
    }
}

void MassSpringDamping::fixEnds(std::shared_ptr<tiago_cable_manipulation::Cable> cable, bool cable_inserted) {
    //ignition::math::Pose3d first_fixed_pose = cable->getLink(0)->WorldPose();
    

    cable->get(0).setFixed(true);
    // cable->getLink(0)->SetWorldPose(first_fixed_pose);
    if(!cable_inserted) { 
        this->end_fixed_pose = cable->getLink(cable->getResolution() - 1)->WorldPose();
    }
    if(cable_inserted) {
        cable->get(cable->getResolution() - 1).setFixed(true);
        cable->getLink(cable->getResolution() - 1)->SetWorldPose(end_fixed_pose);
    }
    
}