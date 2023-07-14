#ifndef SENSOR_READER
#define SENSOR_READER

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <thread>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"

#include <iostream>
#include <geometry_msgs/Wrench.h>

#include "LowPassFilter.hpp"

class SensorReader
{
public:
    SensorReader(ros::Rate rate);

    void FT_sensor_Reading_Callback(const geometry_msgs::WrenchStamped& msg);
    void increaseCounter(){this->i += 1;}
    void resetCounter(){this->i = 0;}

    /*! \brief  Set the flag to start the calibration*/
    void startCalibration(){this->isCalibrationStarted = true;}

    void computeBias(std::vector<double> force_x_readings, std::vector<double> force_y_readings, std::vector<double> force_z_readings, std::vector<double> torque_x_readings, std::vector<double> torque_y_readings, std::vector<double> torque_z_readings);

    /*! \brief  Calibrates the sensor bias averaging the last 100 readings
    \returns the sensor bias*/
    std::vector<double> calibrateSensor();

    /*! \brief  Filter used to smooth fluctuations in the sensor readings
    \returns the average of the last 30 sensor readings*/
    double computeMovingAverage(const std::vector<double>& readings);

    /*! \brief  Transform a force from source frame to target frame
    \returns the transformed force*/
    std::vector<double> tfForceintoBaseLink(std::vector<double> force, std::vector<double> torque, std::string sourceFrame, std::string targetFrame);

    

    //--------------------GETTERS AND SETTERS--------------------//

    void setBias(std::vector<double> bias){this->bias = bias;}
    /*! \brief  Set the flag to end the calibration*/
    void setSensorCalibrated(bool sensor_calibrated){this->isSensorCalibrated = sensor_calibrated;}
    

    std::vector<double> getForceDetected(){return force_detected;}
    std::vector<double> getTorqueDetected(){return torque_detected;}

    std::vector<double> getForce_x_readings(){return force_x_readings;}
    std::vector<double> getForce_y_readings(){return force_y_readings;}
    std::vector<double> getForce_z_readings(){return force_z_readings;}

    std::vector<double> getTorque_x_readings(){return torque_x_readings;}
    std::vector<double> getTorque_y_readings(){return torque_y_readings;}
    std::vector<double> getTorque_z_readings(){return torque_z_readings;}


    /*! \brief  Access the sensor readings after moving average
    \returns filtered force*/
    std::vector<double> getFilteredForce(){
        while (lock){
            ros::spinOnce();
        }
        std::vector<double> force_avg = {Readings_Avg.force_x, Readings_Avg.force_y, Readings_Avg.force_z};
        return force_avg;
    }

    /*! \brief  Access the sensor readings after moving average
    \returns filtered torque*/
    std::vector<double> getFilteredTorque(){
        while (lock){
            ros::spinOnce();
        }
        std::vector<double> torque_avg = {Readings_Avg.torque_x, Readings_Avg.torque_y, Readings_Avg.torque_z};
        return torque_avg;
    }

    /*! \brief  Access the sensor readings after moving average in the base_link frame
    \returns filtered force and torque in base link frame*/
    std::vector<double> getFilteredForce_in_bl(){
        std::vector<double> force_avg = {Readings_Avg.force_x, Readings_Avg.force_y, Readings_Avg.force_z};
        std::vector<double> torque_avg = {Readings_Avg.torque_x, Readings_Avg.torque_y, Readings_Avg.torque_z};
        std::vector<double> bl_force_avg = tfForceintoBaseLink(force_avg, torque_avg, "ur5e_wrist_3_link", "ur5e_base_link");
        return bl_force_avg;
    };


    int getCounter(){return i;}
    std::vector<double> getBias(){return bias;}
    

    /*! \brief  Deallocates the heap memory occupied by the vectors used in the sensor calibration*/
    void clearReadings();

    void printCounter(){
        std::cout << "counter: " << this->i << std::endl;
    }

    struct Reading
    {
        double force_x;
        double force_y;
        double force_z;

        double torque_x;
        double torque_y;
        double torque_z;
    };


    // void pushForce_x(double force_x){this->force_x_readings.push_back(force_x);}
    // void pushForce_y(double force_y){this->force_y_readings.push_back(force_y);}
    // void pushForce_z(double force_z){this->force_z_readings.push_back(force_z);}

    // void pushTorque_x(double torque_x){this->torque_x_readings.push_back(torque_x);}
    // void pushTorque_y(double torque_y){this->torque_y_readings.push_back(torque_y);}
    // void pushTorque_z(double torque_z){this->torque_z_readings.push_back(torque_z);}


    // void setForceDetected(std::vector<double> force_detected){this->force_detected = force_detected;}
    // void setTorqueDetected(std::vector<double> torque_detected){this->torque_detected = torque_detected;}


private:

    std::vector<double> force_detected;
    std::vector<double> torque_detected;
    int i = 0;
    std::vector<double> force_x_readings;
    std::vector<double> force_y_readings;
    std::vector<double> force_z_readings;

    std::vector<double> torque_x_readings;
    std::vector<double> torque_y_readings;
    std::vector<double> torque_z_readings;

    // std::vector<double> force_x_readings_avg;
    // std::vector<double> force_y_readings_avg;
    // std::vector<double> force_z_readings_avg;

    // std::vector<double> torque_x_readings_avg;
    // std::vector<double> torque_y_readings_avg;
    // std::vector<double> torque_z_readings_avg;

    std::vector<double> bias;

    bool isSensorCalibrated = false;
    bool isCalibrationStarted = false;

    const int NUM_READINGS = 30;
    const int number_of_considered_readings = 100;

    Reading Readings_Avg;
    ros::Rate loop_rate;

    tf::TransformListener listener;
    LowPassFilter filter_fx;
    LowPassFilter filter_fy;
    LowPassFilter filter_fz;
    LowPassFilter filter_tx;
    LowPassFilter filter_ty;
    LowPassFilter filter_tz;
    
    bool lock = false;
};


#endif // SENSOR_READER

//PREVIOUS MAIN IMPLEMENTATION

// class SensorReader
// {
// public:
//     SensorReader()
//     {
//         force_detected.resize(3);
//         torque_detected.resize(3);
//         bias.resize(6);

//         for (int i=0; i<3; i++){
//             force_detected[i] = 0;
//             torque_detected[i] = 0;
//         }
//     }
    

//     void setForceDetected(std::vector<double> force_detected){this->force_detected = force_detected;}
//     void setTorqueDetected(std::vector<double> torque_detected){this->torque_detected = torque_detected;}

//     void setBias(std::vector<double> bias){this->bias = bias;}
//     /*! \brief  Set the flag to end the calibration*/
//     void setSensorCalibrated(bool sensor_calibrated){this->isSensorCalibrated = sensor_calibrated;}

//     void setDesiredForce(std::vector<double> desired_force){this->desired_force = desired_force;}
//     void setDesiredTorque(std::vector<double> desired_torque){this->desired_torque = desired_torque;}


//     void increaseCounter(){this->i += 1;}
//     void resetCounter(){this->i = 0;}

//     /*! \brief  Set the flag to start the calibration*/
//     void startCalibration(){this->isCalibrationStarted = true;}

//     void computeBias(std::vector<double> force_x_readings, std::vector<double> force_y_readings, std::vector<double> force_z_readings, std::vector<double> torque_x_readings, std::vector<double> torque_y_readings, std::vector<double> torque_z_readings){
//         int number_of_considered_readings = 100;
//         int startIndex = force_x_readings.size() - number_of_considered_readings;
//         std::vector<double> sum_forces(3);
//         std::vector<double> sum_torques(3);
//         for (int i=startIndex; i<force_x_readings.size(); i++){
//             sum_forces[0] += force_x_readings[i];
//             sum_forces[1] += force_y_readings[i];
//             sum_forces[2] += force_z_readings[i];

//             sum_torques[0] += torque_x_readings[i];
//             sum_torques[1] += torque_y_readings[i];
//             sum_torques[2] += torque_z_readings[i];
//         }
//         std::vector<double> bias = {sum_forces[0]/100, sum_forces[1]/100, sum_forces[2]/100, sum_torques[0]/100, sum_torques[1]/100, sum_torques[2]/100};
//         setBias(bias);
//         sum_forces.clear();
//         sum_torques.clear();
//         sum_forces.shrink_to_fit();
//         sum_torques.shrink_to_fit();
//         // bias.clear();
//         // bias.shrink_to_fit();
//     }

//     /*! \brief  Calibrates the sensor bias averaging the last 100 readings
//     \returns the sensor bias*/
//     std::vector<double> calibrateSensor(){
//         int counter_atStart = getCounter();
//         int i = getCounter();
//         // printCounter();
//         // std::cout<<"--------------------"<<std::endl;
//         int number_of_considered_readings = 100;
//         while (i<counter_atStart+number_of_considered_readings){
//             i = getCounter();
//             // printCounter();
//         }
//         std::vector<double> force_x_readings = getForce_x_readings();
//         std::vector<double> force_y_readings = getForce_y_readings();
//         std::vector<double> force_z_readings = getForce_z_readings();

//         std::vector<double> torque_x_readings = getTorque_x_readings();
//         std::vector<double> torque_y_readings = getTorque_y_readings();
//         std::vector<double> torque_z_readings = getTorque_z_readings();

//         computeBias(force_x_readings, force_y_readings, force_z_readings, torque_x_readings, torque_y_readings, torque_z_readings);
//         std::vector<double> bias = getBias();
//         return bias;
//     }

//     double computeMovingAverage(const std::vector<double>& readings) {
//         double sum = 0.0;
//         for (double reading : readings) {
//             sum += reading;
//         }
//         return sum / readings.size();
//     }

//     void pushForce_x(double force_x){this->force_x_readings.push_back(force_x);}
//     void pushForce_y(double force_y){this->force_y_readings.push_back(force_y);}
//     void pushForce_z(double force_z){this->force_z_readings.push_back(force_z);}

//     void pushTorque_x(double torque_x){this->torque_x_readings.push_back(torque_x);}
//     void pushTorque_y(double torque_y){this->torque_y_readings.push_back(torque_y);}
//     void pushTorque_z(double torque_z){this->torque_z_readings.push_back(torque_z);}

//     std::vector<double> getForceDetected(){return force_detected;}
//     std::vector<double> getTorqueDetected(){return torque_detected;}

//     std::vector<double> getForce_x_readings(){return force_x_readings;}
//     std::vector<double> getForce_y_readings(){return force_y_readings;}
//     std::vector<double> getForce_z_readings(){return force_z_readings;}

//     std::vector<double> getTorque_x_readings(){return torque_x_readings;}
//     std::vector<double> getTorque_y_readings(){return torque_y_readings;}
//     std::vector<double> getTorque_z_readings(){return torque_z_readings;}

//     std::vector<double> getDesiredForce(){return desired_force;}
//     std::vector<double> getDesiredTorque(){return desired_torque;}

//     std::vector<double> getFilteredForce(){
//         std::vector<double> force_avg = {Readings_Avg.force_x, Readings_Avg.force_y, Readings_Avg.force_z};
//         return force_avg;
//     }

//     std::vector<double> getFilteredTorque(){
//         std::vector<double> torque_avg = {Readings_Avg.torque_x, Readings_Avg.torque_y, Readings_Avg.torque_z};
//         return torque_avg;
//     }

//     int getCounter(){return i;}
//     std::vector<double> getBias(){return bias;}

//     void FT_sensor_Reading_Callback(const geometry_msgs::WrenchStamped& msg){
//         setForceDetected({msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z});
//         setTorqueDetected({msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z});
        
//         if (!isSensorCalibrated && isCalibrationStarted) {
//             increaseCounter();
//             // printCounter();

//             pushForce_x(msg.wrench.force.x);
//             pushForce_y(msg.wrench.force.y);
//             pushForce_z(msg.wrench.force.z);

//             pushTorque_x(msg.wrench.torque.x);
//             pushTorque_y(msg.wrench.torque.y);
//             pushTorque_z(msg.wrench.torque.z);
//         }

//         if (isSensorCalibrated) {
//             pushForce_x(msg.wrench.force.x);
//             pushForce_y(msg.wrench.force.y);
//             pushForce_z(msg.wrench.force.z);

//             pushTorque_x(msg.wrench.torque.x);
//             pushTorque_y(msg.wrench.torque.y);
//             pushTorque_z(msg.wrench.torque.z);

//             if (force_x_readings.size() > NUM_READINGS) {
//                 force_x_readings.erase(force_x_readings.begin());
//                 force_y_readings.erase(force_y_readings.begin());
//                 force_z_readings.erase(force_z_readings.begin());

//                 torque_x_readings.erase(torque_x_readings.begin());
//                 torque_y_readings.erase(torque_y_readings.begin());
//                 torque_z_readings.erase(torque_z_readings.begin());
//             }

//             Readings_Avg.force_x = computeMovingAverage(force_x_readings);
//             Readings_Avg.force_y = computeMovingAverage(force_y_readings);
//             Readings_Avg.force_z = computeMovingAverage(force_z_readings);

//             Readings_Avg.torque_x = computeMovingAverage(torque_x_readings);
//             Readings_Avg.torque_y = computeMovingAverage(torque_y_readings);
//             Readings_Avg.torque_z = computeMovingAverage(torque_z_readings);

//         }


//     }

//     /*! \brief  Deallocates the heap memory occupied by the vectors used in the sensor calibration*/
//     void clearReadings(){
//         force_x_readings.clear();
//         force_y_readings.clear();
//         force_z_readings.clear();

//         force_x_readings.shrink_to_fit();
//         force_y_readings.shrink_to_fit();
//         force_z_readings.shrink_to_fit();

//         torque_x_readings.clear();
//         torque_y_readings.clear();
//         torque_z_readings.clear();

//         torque_x_readings.shrink_to_fit();
//         torque_y_readings.shrink_to_fit();
//         torque_z_readings.shrink_to_fit();

//         // std::cout<<"SIZE: "<< force_x_readings.size()<<std::endl;

//         // bias.shrink_to_fit();

//         resetCounter();
//     }

//     void printCounter(){
//         std::cout << "counter: " << this->i << std::endl;
//     }

//     struct Reading
//     {
//         double force_x;
//         double force_y;
//         double force_z;

//         double torque_x;
//         double torque_y;
//         double torque_z;
//     };

// private:
//     std::vector<double> force_detected;
//     std::vector<double> torque_detected;
//     int i = 0;
//     std::vector<double> force_x_readings;
//     std::vector<double> force_y_readings;
//     std::vector<double> force_z_readings;

//     std::vector<double> torque_x_readings;
//     std::vector<double> torque_y_readings;
//     std::vector<double> torque_z_readings;

//     std::vector<double> bias;

//     std::vector<double> desired_force = {11, -21.16, 0};
//     std::vector<double> desired_torque = {4.3, 5.7, 0.17};

//     bool isSensorCalibrated = false;
//     bool isCalibrationStarted = false;

//     const int NUM_READINGS = 30;
//     // std::array<double, 6> Readings_Avg;

//     Reading Readings_Avg;
    
// };
