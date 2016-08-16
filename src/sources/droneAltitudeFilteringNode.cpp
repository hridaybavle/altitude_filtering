/*
 *  Created on: 22.06.2016.
 *      Author: Zorana Milosevic
 */

#include "ros/ros.h"
#include "droneAltitudeFiltering.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "DroneAltitudeFiltering");
//    ros::AsyncSpinner spinner(1);
    ros::NodeHandle n;

    //Init
    std::cout<<"Starting DroneAltitudeFiltering..."<<std::endl;

    //DroneAltitudeFiltering
    DroneAltitudeFiltering MyDroneAltitudeFiltering;
   // MyDroneAltitudeFiltering.open(n,"DroneAltitudeFiltering");
    MyDroneAltitudeFiltering.open(n);

    try
    {
         while(ros::ok())
         {
        //Read messages
        ros::spinOnce();

//        spinner.start();
          if(MyDroneAltitudeFiltering.run())
          {

          }
          MyDroneAltitudeFiltering.sleep();
//        spinner.stop();
         }
    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }

    return 0;

}

