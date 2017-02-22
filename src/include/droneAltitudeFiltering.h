/*
 *  Created on: 22.06.2016.
 *      Author: Zorana Milosevic
 */


#ifndef DRONE_ALTITUDE_FILTERING_H
#define DRONE_ALTITUDE_FILTERING_H


// C++ standar libraries
#include <cstdlib>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <set>
#include <algorithm>
#include <sstream>
#include <string>
#include <cmath>
#include <numeric>

// ROS
#include "ros/ros.h"

#include "sensor_msgs/Range.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Temperature.h"
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64.h"

//mavros
#include "mavros_msgs/Altitude.h"

// DroneModule parent class
#include "droneModuleROS.h"

// droneMsgsROS
#include "droneMsgsROS/droneNavData.h"
#include <droneMsgsROS/droneAltitude.h>
#include "droneMsgsROS/droneStatus.h"

//std_msgs
#include "std_msgs/Int32.h"

//Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#define BUFFER_SIZE 3
#define ALTITUDE_THRESHOLD 0.10
#define OBJECT_THRESHOLD 0.20
#define ACCELERATIONS_COUNT 80


const double Pressure_sea_level = 101325;

class DroneAltitudeFiltering: public DroneModule
{

private:
    //ros::Publisher droneFilteredAltitudePub;

public:
    DroneAltitudeFiltering();
    ~DroneAltitudeFiltering();
    //void open(ros::NodeHandle & nIn, std::string moduleName);
    void close();
    geometry_msgs::PoseStamped altitudeData;
    geometry_msgs::PoseStamped barometerData;
    geometry_msgs::PoseStamped objectHeightEstData;
    geometry_msgs::PoseStamped objectHeightData;
    geometry_msgs::PoseStamped accelerationData;

    void droneLidarCallbackSim(const geometry_msgs::PoseStamped& msg);
    void droneLidarCallbackReal(const sensor_msgs::Range &msg);
    void droneImuCallback(const sensor_msgs::Imu &msg);
    void droneRotationAnglesCallback(const geometry_msgs::Vector3Stamped &msg);
    void droneAtmPressureCallback(const sensor_msgs::FluidPressure &msg);
    void droneTemperatureCallback(const sensor_msgs::Temperature &msg);
    void droneMavrosAltitudeCallback(const mavros_msgs::Altitude &msg);
    void droneStatusCallback(const droneMsgsROS::droneStatus &msg);

    void open(ros::NodeHandle & nIn);

    void PublishAltitudeData(const geometry_msgs::PoseStamped &altitudemsg);
    void OpenModel();

    float angular_velocity, linear_acceleration_z, avg_linear_acceleration_z;
    float pitch_angle;
    float atm_pressure, temperature, barometer_height, first_barometer_height;
    float first_measured_lidar_altitude;
    float counter, count, stop_count, object_counter;
    std::vector<double> lidar_measurements;
    int peak_counter;
    double prev_mean;
    double object_height;

    //    double Pb;
    //    double hb;
    //    double R_as;
    //    double G0;
    //    double Lb;
    //    double Tb;
    //    double P;
    //    double M;
    //    double nn, nd, ndiff,ndiv, d;
    double timePrev, timeNow;
    double deltaT;
    std::vector<bool> measurement_activation;


protected:
    bool init();
    //void readParameters();
    ros::Subscriber droneLidarSim;
    ros::Subscriber droneLidarReal;
    ros::Subscriber droneImuSub;
    ros::Subscriber droneRotationAnglesSub;
    ros::Subscriber droneAtmPressureSub;
    ros::Subscriber droneTemperatureSub;
    ros::Subscriber droneMavrosAltitudeSub;
    ros::Subscriber droneStatusSub;

    ros::Publisher droneAltitudePub;
    ros::Publisher droneBarometerHeightPub;
    ros::Publisher droneEstObjectHeightPub;
    ros::Publisher droneObjectHeightPub;
    ros::Publisher droneAccelerationsPub;
    ros::Publisher droneMahaDistancePub;

public:
    bool run();
    bool resetValues();
    bool startVal();


public:
    float measuredAltitude, lastMeasuredAltitude, filteredAltitude, altitude_treshold;
    bool object_below;
    droneMsgsROS::droneStatus droneStatus;
};


#endif

