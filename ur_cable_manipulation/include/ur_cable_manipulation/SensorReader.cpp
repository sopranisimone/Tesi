#include "SensorReader.hpp"

SensorReader::SensorReader(ros::Rate rate) : loop_rate(rate), filter_fx(0.0,0.1), filter_fy(0.0,0.1), filter_fz(0.0,0.1), filter_tx(0.0,0.1), filter_ty(0.0,0.1), filter_tz(0.0,0.1) {
    
    force_detected.resize(3);
    torque_detected.resize(3);
    bias.resize(6);

    for (int i=0; i<3; i++){
        force_detected[i] = 0;
        torque_detected[i] = 0;
    }
}


void SensorReader::FT_sensor_Reading_Callback(const geometry_msgs::WrenchStamped& msg){
        // this->setForceDetected({msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z});
        // this->setTorqueDetected({msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z});

        force_detected = {msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z};
        torque_detected = {msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z};
        
        if (!isSensorCalibrated && isCalibrationStarted) {
            // this->increaseCounter();
            // printCounter();

            // this->pushForce_x(msg.wrench.force.x);
            // this->pushForce_y(msg.wrench.force.y);
            // this->pushForce_z(msg.wrench.force.z);

            // this->pushTorque_x(msg.wrench.torque.x);
            // this->pushTorque_y(msg.wrench.torque.y);
            // this->pushTorque_z(msg.wrench.torque.z);

            force_x_readings.push_back(msg.wrench.force.x);
            force_y_readings.push_back(msg.wrench.force.y);
            force_z_readings.push_back(msg.wrench.force.z);

            torque_x_readings.push_back(msg.wrench.torque.x);
            torque_y_readings.push_back(msg.wrench.torque.y);
            torque_z_readings.push_back(msg.wrench.torque.z);
            
        }

        if (isSensorCalibrated) {
            // this->pushForce_x(msg.wrench.force.x);
            // this->pushForce_y(msg.wrench.force.y);
            // this->pushForce_z(msg.wrench.force.z);

            // this->pushTorque_x(msg.wrench.torque.x);
            // this->pushTorque_y(msg.wrench.torque.y);
            // this->pushTorque_z(msg.wrench.torque.z);

            force_x_readings.push_back(msg.wrench.force.x);
            force_y_readings.push_back(msg.wrench.force.y);
            force_z_readings.push_back(msg.wrench.force.z);

            torque_x_readings.push_back(msg.wrench.torque.x);
            torque_y_readings.push_back(msg.wrench.torque.y);
            torque_z_readings.push_back(msg.wrench.torque.z);
            

            if (force_x_readings.size() > NUM_READINGS) {
                force_x_readings.erase(force_x_readings.begin());
                force_y_readings.erase(force_y_readings.begin());
                force_z_readings.erase(force_z_readings.begin());

                torque_x_readings.erase(torque_x_readings.begin());
                torque_y_readings.erase(torque_y_readings.begin());
                torque_z_readings.erase(torque_z_readings.begin());
            }

            lock = true;

            //moving averaege
            Readings_Avg.force_x = this->computeMovingAverage(force_x_readings);
            Readings_Avg.force_y = this->computeMovingAverage(force_y_readings);
            Readings_Avg.force_z = this->computeMovingAverage(force_z_readings);

            Readings_Avg.torque_x = this->computeMovingAverage(torque_x_readings);
            Readings_Avg.torque_y = this->computeMovingAverage(torque_y_readings);
            Readings_Avg.torque_z = this->computeMovingAverage(torque_z_readings);

            ROS_INFO("force_x: %f\n", Readings_Avg.force_x);
            ROS_INFO("force_y: %f\n", Readings_Avg.force_y);
            ROS_INFO("force_z: %f\n", Readings_Avg.force_z);

            ROS_INFO("torque_x: %f\n", Readings_Avg.torque_x);
            ROS_INFO("torque_y: %f\n", Readings_Avg.torque_y);
            ROS_INFO("torque_z: %f\n", Readings_Avg.torque_z);


            //leaky integrator low pass filter
            filter_fx.filter(Readings_Avg.force_x);
            filter_fy.filter(Readings_Avg.force_y);
            filter_fz.filter(Readings_Avg.force_z);

            filter_tx.filter(Readings_Avg.torque_x);
            filter_ty.filter(Readings_Avg.torque_y);
            filter_tz.filter(Readings_Avg.torque_z);

            Readings_Avg.force_x = filter_fx.getValue();
            Readings_Avg.force_y = filter_fy.getValue();
            Readings_Avg.force_z = filter_fz.getValue();

            Readings_Avg.torque_x = filter_tx.getValue();
            Readings_Avg.torque_y = filter_ty.getValue();
            Readings_Avg.torque_z = filter_tz.getValue();

            ROS_INFO("force_x filter: %f\n", Readings_Avg.force_x);
            ROS_INFO("force_y filter: %f\n", Readings_Avg.force_y);
            ROS_INFO("force_z filter: %f\n", Readings_Avg.force_z);

            ROS_INFO("torque_x filter: %f\n", Readings_Avg.torque_x);
            ROS_INFO("torque_y filter: %f\n", Readings_Avg.torque_y);
            ROS_INFO("torque_z filter: %f\n", Readings_Avg.torque_z);


            lock = false;


            
            // force_x_readings_avg.push_back(this->computeMovingAverage(force_x_readings));
            // force_y_readings_avg.push_back(this->computeMovingAverage(force_y_readings));
            // force_z_readings_avg.push_back(this->computeMovingAverage(force_z_readings));

            // torque_x_readings_avg.push_back(this->computeMovingAverage(torque_x_readings));
            // torque_y_readings_avg.push_back(this->computeMovingAverage(torque_y_readings));
            // torque_z_readings_avg.push_back(this->computeMovingAverage(torque_z_readings));

            // if (force_x_readings_avg.size() > NUM_READINGS){
            //     force_x_readings_avg.erase(force_x_readings_avg.begin());
            //     force_y_readings_avg.erase(force_y_readings_avg.begin());
            //     force_z_readings_avg.erase(force_z_readings_avg.begin());

            //     torque_x_readings_avg.erase(torque_x_readings_avg.begin());
            //     torque_y_readings_avg.erase(torque_y_readings_avg.begin());
            //     torque_z_readings_avg.erase(torque_z_readings_avg.begin());
            // }

            // for (const double input : force_x_readings_avg) {
            //     filter_fx.filter(input);
            //     std::cout << "Filtered value: " << filter_fx.getValue() << std::endl;
            // }
            // Readings_Avg.force_x = filter_fx.getValue();

            // for (const double input : force_y_readings_avg) {
            //     filter_fy.filter(input);
            //     std::cout << "Filtered value: " << filter_fy.getValue() << std::endl;
            // }
            // Readings_Avg.force_y = filter_fy.getValue();

            // for (const double input : force_z_readings_avg) {
            //     filter_fz.filter(input);
            //     std::cout << "Filtered value: " << filter_fz.getValue() << std::endl;
            // }
            // Readings_Avg.force_z = filter_fz.getValue();

            // for (const double input : torque_x_readings_avg) {
            //     filter_tx.filter(input);
            //     std::cout << "Filtered value: " << filter_tx.getValue() << std::endl;
            // }
            // Readings_Avg.torque_x = filter_tx.getValue();

            // for (const double input : torque_y_readings_avg) {
            //     filter_ty.filter(input);
            //     std::cout << "Filtered value: " << filter_ty.getValue() << std::endl;
            // }
            // Readings_Avg.torque_y = filter_ty.getValue();

            // for (const double input : torque_z_readings_avg) {
            //     filter_tz.filter(input);
            //     std::cout << "Filtered value: " << filter_tz.getValue() << std::endl;
            // }
            // Readings_Avg.torque_z = filter_tz.getValue();

        }
            //think if compute moving average result needs to be overwritten to the last force x reading element 
            //in order to low pass filter 30 force x readings and not needing to save reading avgs memory
            //low pass filter here

    }


void SensorReader::computeBias(std::vector<double> force_x_readings, std::vector<double> force_y_readings, std::vector<double> force_z_readings, std::vector<double> torque_x_readings, std::vector<double> torque_y_readings, std::vector<double> torque_z_readings){
        // int number_of_considered_readings = 100;
        int startIndex = 0;
        int endIndex = force_x_readings.size();
        if (endIndex > number_of_considered_readings) 
            startIndex = endIndex - number_of_considered_readings;
        // std::cout << "startIndex: \n" << startIndex << std::endl;
        // std::cout << "endIndex: \n" << endIndex << std::endl;
        std::vector<double> sum_forces(3);
        std::vector<double> sum_torques(3);
        try {
            for (int i=startIndex; i<endIndex; i++){
                sum_forces[0] += force_x_readings[i];
                sum_forces[1] += force_y_readings[i];
                sum_forces[2] += force_z_readings[i];

                sum_torques[0] += torque_x_readings[i];
                sum_torques[1] += torque_y_readings[i];
                sum_torques[2] += torque_z_readings[i];
            }
        } catch (const std::exception& e) {
            std::cout << e.what() << std::endl;
        }
        // std::cout<< "Size of force_x_readings in computeBias: " << force_x_readings.size() << std::endl;
        // std::cout<< "Size of force_y_readings in computeBias: " << force_y_readings.size() << std::endl;
        // std::cout<< "Size of force_z_readings in computeBias: " << force_z_readings.size() << std::endl;
        // std::cout<< "Size of torque_x_readings in computeBias: " << torque_x_readings.size() << std::endl;
        // std::cout<< "Size of torque_y_readings in computeBias: " << torque_y_readings.size() << std::endl;
        // std::cout<< "Size of torque_z_readings in computeBias: " << torque_z_readings.size() << std::endl;
        // std::cout<< "Size of sum_forces in computeBias: " << sum_forces.size() << std::endl;
        // std::cout<< "Size of sum_torques in computeBias: " << sum_torques.size() << std::endl;

        
        std::vector<double> bias_force = {sum_forces[0]/(endIndex-startIndex), sum_forces[1]/(endIndex-startIndex), sum_forces[2]/(endIndex-startIndex)};
        std::vector<double> bias_torque = {sum_torques[0]/(endIndex-startIndex), sum_torques[1]/(endIndex-startIndex), sum_torques[2]/(endIndex-startIndex)};
        // std::vector<double> bias = {bias_force[0], bias_force[1], bias_force[2], bias_torque[0], bias_torque[1], bias_torque[2]};

        // std::cout<< "Size of bias_force in computeBias: " << bias_force.size() << std::endl;
        // std::cout<< "Size of bias_torque in computeBias: " << bias_torque.size() << std::endl;
        
        std::vector<double> bias = tfForceintoBaseLink(bias_force, bias_torque, "ur5e_wrist_3_link", "ur5e_base_link");
        this->setBias(bias);
        sum_forces.clear();
        sum_torques.clear();
        sum_forces.shrink_to_fit();
        sum_torques.shrink_to_fit();
    }


std::vector<double> SensorReader::calibrateSensor(){
        int counter_atStart = this->getCounter();
        int i = this->getCounter();
        // printCounter();
        // std::cout<<"--------------------"<<std::endl;
        // int number_of_considered_readings = 100;
        // while (i<counter_atStart+number_of_considered_readings){
        //     i = this->getCounter();
        //     // printCounter();
        // }
        std::cout<<"--------------------"<<std::endl;
        while (force_x_readings.size() < number_of_considered_readings){
            // i = this->getCounter();
            // printCounter();
            // std::cout<<"Size of force_x_readings: "<<force_x_readings.size()<<std::endl;
            loop_rate.sleep();
        }
        // std::vector<double> force_x_readings = this->getForce_x_readings();
        // std::vector<double> force_y_readings = this->getForce_y_readings();
        // std::vector<double> force_z_readings = this->getForce_z_readings();

        // std::vector<double> torque_x_readings = this->getTorque_x_readings();
        // std::vector<double> torque_y_readings = this->getTorque_y_readings();
        // std::vector<double> torque_z_readings = this->getTorque_z_readings();

        this->computeBias(force_x_readings, force_y_readings, force_z_readings, torque_x_readings, torque_y_readings, torque_z_readings);
        // std::vector<double> bias = this->getBias();
        return bias;
    }

double SensorReader::computeMovingAverage(const std::vector<double>& readings) {
        double sum = 0.0;
        for (double reading : readings) {
            sum += reading;
        }
        return sum / readings.size();
    }


void SensorReader::clearReadings(){
        force_x_readings.clear();
        force_y_readings.clear();
        force_z_readings.clear();

        force_x_readings.shrink_to_fit();
        force_y_readings.shrink_to_fit();
        force_z_readings.shrink_to_fit();

        torque_x_readings.clear();
        torque_y_readings.clear();
        torque_z_readings.clear();

        torque_x_readings.shrink_to_fit();
        torque_y_readings.shrink_to_fit();
        torque_z_readings.shrink_to_fit();

        // std::cout<<" FORCE X SIZE AFTER CLEAR: "<< force_x_readings.size()<<std::endl;
        // std::cout<<" FORCE Y SIZE AFTER CLEAR: "<< force_y_readings.size()<<std::endl;
        // std::cout<<" FORCE Z SIZE AFTER CLEAR: "<< force_z_readings.size()<<std::endl;
        // std::cout<<" TORQUE X SIZE AFTER CLEAR: "<< torque_x_readings.size()<<std::endl;
        // std::cout<<" TORQUE Y SIZE AFTER CLEAR: "<< torque_y_readings.size()<<std::endl;
        // std::cout<<" TORQUE Z SIZE AFTER CLEAR: "<< torque_z_readings.size()<<std::endl;

        // std::cout<<"Counter before clear: "<<this->getCounter()<<std::endl;

        // bias.shrink_to_fit();

        this->resetCounter();

        // std::cout<<"Counter after clear: "<<this->getCounter()<<std::endl;
    }

    std::vector<double> SensorReader::tfForceintoBaseLink(std::vector<double> force, std::vector<double> torque, std::string sourceFrame, std::string targetFrame){

        // tf::TransformListener listener;
        // ros::Duration(0.05).sleep();  //it needs some time to initialize the buffer 

        // std::string sourceFrame = "ur5e_wrist_3_link";
        // std::string targetFrame = "ur5e_base_link";

        // Create a wrench expressed in the source frame
        geometry_msgs::WrenchStamped sourceWrench;
        sourceWrench.header.frame_id = sourceFrame;
        sourceWrench.wrench.force.x = force[0];
        sourceWrench.wrench.force.y = force[1];
        sourceWrench.wrench.force.z = force[2];
        sourceWrench.wrench.torque.x = torque[0];
        sourceWrench.wrench.torque.y = torque[1];
        sourceWrench.wrench.torque.z = torque[2];

        try {
            // Transform the force component
            tf::StampedTransform forceTransform;
            listener.lookupTransform(targetFrame, sourceFrame, ros::Time(0), forceTransform);
            tf::Vector3 transformedForce = forceTransform.getBasis() * tf::Vector3(sourceWrench.wrench.force.x,
                                                                                sourceWrench.wrench.force.y,
                                                                                sourceWrench.wrench.force.z);

            // Transform the torque component
            tf::StampedTransform torqueTransform;
            listener.lookupTransform(targetFrame, sourceFrame, ros::Time(0), torqueTransform);
            tf::Vector3 transformedTorque = torqueTransform.getBasis() * tf::Vector3(sourceWrench.wrench.torque.x,
                                                                                    sourceWrench.wrench.torque.y,
                                                                                    sourceWrench.wrench.torque.z);

            // Create a transformed wrench in the target frame
            geometry_msgs::WrenchStamped targetWrench;
            targetWrench.header.frame_id = targetFrame;
            targetWrench.wrench.force.x = transformedForce.getX();
            targetWrench.wrench.force.y = transformedForce.getY();
            targetWrench.wrench.force.z = transformedForce.getZ();
            targetWrench.wrench.torque.x = transformedTorque.getX();
            targetWrench.wrench.torque.y = transformedTorque.getY();
            targetWrench.wrench.torque.z = transformedTorque.getZ();

            std::vector<double> transformed_wrench = {targetWrench.wrench.force.x, targetWrench.wrench.force.y, targetWrench.wrench.force.z, targetWrench.wrench.torque.x, targetWrench.wrench.torque.y, targetWrench.wrench.torque.z};

            // Print the original wrench
            // std::cout << "Original wrench (" << sourceFrame << "): " << std::endl;
            // std::cout << "Force: (" << sourceWrench.wrench.force.x << ", "
            //         << sourceWrench.wrench.force.y << ", "
            //         << sourceWrench.wrench.force.z << ")" << std::endl;
            // std::cout << "Torque: (" << sourceWrench.wrench.torque.x << ", "
            //         << sourceWrench.wrench.torque.y << ", "
            //         << sourceWrench.wrench.torque.z << ")" << std::endl;

            // // Print the transformed wrench
            // std::cout << "Transformed wrench (base_link): " << std::endl;
            // std::cout << "Force: (" << targetWrench.wrench.force.x << ", "
            //         << targetWrench.wrench.force.y << ", "
            //         << targetWrench.wrench.force.z << ")" << std::endl;
            // std::cout << "Torque: (" << targetWrench.wrench.torque.x << ", "
            //         << targetWrench.wrench.torque.y << ", "
            //         << targetWrench.wrench.torque.z << ")\n" << std::endl;

            return transformed_wrench;

        } catch (tf::TransformException& ex) {
            ROS_ERROR("Failed to transform wrench: %s", ex.what());

            return {0,0,0,0,0,0};
        }
        
    }
