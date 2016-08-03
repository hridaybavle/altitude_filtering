/*
 *  Created on: 22.06.2016.
 *      Author: Zorana Milosevic
 */


#include "droneAltitudeFiltering.h"

#include <iostream>
#include <fstream>

// using namespaces only in cpp files!!
using namespace std;





string ime = "/home/zorana/workspace/ros/quadrotor_stack_catkin_ws/altitudeZaSlanje.txt";
string ime1 = "/home/zorana/workspace/ros/quadrotor_stack_catkin_ws/altitudeSenzor.txt";
string imeZ = "/home/zorana/workspace/ros/quadrotor_stack_catkin_ws/nesto.txt";
string imeN = "/home/zorana/workspace/ros/quadrotor_stack_catkin_ws/altitudeF.txt";


ofstream myfileA(ime.c_str());
ofstream myfileS(ime1.c_str());
ofstream myfileZ(imeZ.c_str());
ofstream myfileOut(imeN.c_str());


DroneAltitudeFiltering::DroneAltitudeFiltering() : DroneModule(droneModule::active)
{
    if(!init())
        std::cout<<"Error init"<<std::endl;

    return;
}


DroneAltitudeFiltering::~DroneAltitudeFiltering()
{
    close();
    return;
}



void DroneAltitudeFiltering::open(ros::NodeHandle & nIn)
{
    DroneModule::open(nIn);

    droneLidar               = n.subscribe("altitude", 1, &DroneAltitudeFiltering::droneLidarCallback, this);
    droneLidar1               = n.subscribe("/px4flow/raw/px4Flow_pose_z", 1, &DroneAltitudeFiltering::droneLidarCallback1, this);


    droneAltitudePub = n.advertise<geometry_msgs::PoseStamped>("altitudeF", 1, true);
    droneObjectDetectedPub = n.advertise<std_msgs::Int8>("object_detected", 1, true);

    init();

    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;

    return;
}





bool DroneAltitudeFiltering::init()
{
    measuredAltitude=filteredAltitude=0.0;
    altitude_treshold = 0.25;
    object_detected.data = 0;

    return true;
}


void DroneAltitudeFiltering::close()
{
    DroneModule::close();
    return;
}





bool DroneAltitudeFiltering::run()
{
    if(!DroneModule::run())
        return false;

    if(droneModuleOpened==false)
        return false;

    return true;
}

void DroneAltitudeFiltering::droneLidarCallback( const droneMsgsROS::droneAltitude &msg)
{
    //cout << -msg.altitude << endl;
    myfileA << -msg.altitude << endl;
}

void DroneAltitudeFiltering::droneLidarCallback1( const geometry_msgs::PoseStamped &msg)
{
    myfileS << msg.pose.position.z << endl;

    measuredAltitude = msg.pose.position.z;
//    int p;
//    p=object_detected.data;
//    switch (p)
//    {
//    case 0:
//        if (abs(measuredAltitude - lastMeasuredAltitude)>altitude_treshold) {
//            object_detected.data = 1;
//            object_height = (measuredAltitude - lastMeasuredAltitude);
//            filteredAltitude = lastMeasuredAltitude;
//        }
//        else
//            filteredAltitude = measuredAltitude;
//        break;
//    case 1:
//        if (abs(measuredAltitude - lastMeasuredAltitude)>altitude_treshold) {
//            object_detected.data = 0;
//            filteredAltitude = measuredAltitude;
//        }
//        else
//            filteredAltitude = measuredAltitude - object_height;
//        break;
//    }

//    altitudeData.pose.position.z = filteredAltitude;

//    PublishObjectDetectedData(object_detected);


//    myfileOut << filteredAltitude <<endl;

//    PublishAltitudeData(altitudeData);
//    myfileZ<<p<<endl;

     if (abs(measuredAltitude - lastMeasuredAltitude)>altitude_treshold)
     {
         object_height =+ lastMeasuredAltitude - measuredAltitude;

         if(abs(object_height) < 0.05){
            object_below = false;
         }
         else {
            object_below = true;
         }

     }

     if(object_below){
         filteredAltitude = measuredAltitude + object_height;
     }
     else {
         filteredAltitude = measuredAltitude;
     }


    lastMeasuredAltitude=msg.pose.position.z;

}


void DroneAltitudeFiltering::PublishAltitudeData(const geometry_msgs::PoseStamped &altitudemsg)
{
     if(moduleStarted == true)
       droneAltitudePub.publish(altitudemsg);
     return;
}

void DroneAltitudeFiltering::PublishObjectDetectedData(const std_msgs::Int8& objectDetectedmsg)
{
    if(moduleStarted == true)
        droneObjectDetectedPub.publish(objectDetectedmsg);
    return;
}

