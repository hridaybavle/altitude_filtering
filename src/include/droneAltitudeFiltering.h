/*
 *  Created on: 22.06.2016.
 *      Author: Zorana Milosevic
 */


#ifndef DRONE_ALTITUDE_FILTERING_H
#define DRONE_ALTITUDE_FILTERING_H

#include "ros/ros.h"

#include "opencv2/highgui/highgui.hpp"
#include <nav_msgs/Odometry.h>
#include <droneMsgsROS/droneAltitude.h>

#include "droneModuleROS.h"

#include "geometry_msgs/PoseStamped.h"

#include <vector>

#include "std_msgs/Int8.h"


#include <sstream>


// ROS
#include "ros/ros.h"

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

// DroneModule parent class
#include "droneModuleROS.h"

// droneMsgsROS
#include "droneMsgsROS/droneNavData.h"
#include "droneMsgsROS/societyPose.h"
#include "droneMsgsROS/droneInfo.h"

// Messages
#include "std_msgs/Int32.h"


//#include "otherswarmagentlistener.h"




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

    void droneLidarCallback(const droneMsgsROS::droneAltitude& msg);


    void droneLidarCallback1(const geometry_msgs::PoseStamped& msg);


    void open(ros::NodeHandle & nIn);


    void PublishAltitudeData(const geometry_msgs::PoseStamped &altitudemsg);
    void PublishObjectDetectedData(const std_msgs::Int8& objectDetectedmsg);

    //int object_detected = 0;
    std_msgs::Int8 object_detected;
    float object_height;



protected:
    bool init();
    //void readParameters();
    ros::Subscriber droneLidar;
    ros::Subscriber droneLidar1;

    ros::Publisher droneAltitudePub;
    ros::Publisher droneObjectDetectedPub;




public:
    bool run();


public:
    float measuredAltitude, lastMeasuredAltitude, filteredAltitude, altitude_treshold;
};


#endif
