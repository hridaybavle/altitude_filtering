/*
 *  Created on: 22.06.2016.
 *      Author: Zorana Milosevic
 */


#include "droneAltitudeFiltering.h"

#include <iostream>
#include <fstream>

// using namespaces only in cpp files!!
using namespace std;

Eigen::MatrixXd x_kk(7,1),p_kk(7,7), T(7,7),  F(7,7);
Eigen::MatrixXd H(5,7), R(5,5);
Eigen::MatrixXd x_k1k(7,1), p_k1k(7,7);
Eigen::MatrixXd z_est_k(5,1);
Eigen::MatrixXd v(5,1), S(5,5);
Eigen::MatrixXd dis_mahala(5,1);
Eigen::MatrixXd K(7,5);
Eigen::MatrixXd x_k1k1(7,1), p_k1k1(7,7);
Eigen::MatrixXd I(7,7);


DroneAltitudeFiltering::DroneAltitudeFiltering() : DroneModule(droneModule::active, 100)
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

    droneLidarSim            = n.subscribe("/px4flow/raw/px4Flow_pose_z", 1, &DroneAltitudeFiltering::droneLidarCallbackSim, this);
    droneLidarReal           = n.subscribe("mavros/distance_sensor/hrlv_ez4_pub",1, &DroneAltitudeFiltering::droneLidarCallbackReal, this);
    droneImuSub              = n.subscribe("mavros/imu/data",1,&DroneAltitudeFiltering::droneImuCallback, this);
    droneRotationAnglesSub   = n.subscribe("rotation_angles",1, &DroneAltitudeFiltering::droneRotationAnglesCallback, this);
    droneAtmPressureSub      = n.subscribe("mavros/imu/atm_pressure",1,&DroneAltitudeFiltering::droneAtmPressureCallback, this);
    droneTemperatureSub      = n.subscribe("mavros/imu/temperature",1,&DroneAltitudeFiltering::droneTemperatureCallback, this);

    droneAltitudePub         = n.advertise<geometry_msgs::PoseStamped>("altitudeFiltered", 1, true);

    init();

    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;

    OpenModel();

    return;
}


bool DroneAltitudeFiltering::init()
{
    //measuredAltitude=filteredAltitude=0.0;
    altitude_treshold  = 0.25;
    object_height      = 0;
    counter            = 0;
    x_kk.setZero(7,1),p_kk.setZero(7,7), T.setZero(7,7),  F.setZero(7,7);
    H.setZero(5,7), R.setZero(5,5);
    x_k1k.setZero(7,1), p_k1k.setZero(7,7);
    z_est_k.setZero(5,1);
    v.setZero(5,1), S.setZero(5,5);
    K.setZero(7,5);
    x_k1k1.setZero(7,1), p_k1k1.setZero(7,7);
    I.setZero(7,7);


    measuredAltitude       = 0;
    linear_acceleration_z  = 0;
    angular_velocity       = 0;
    barometer_height       = 0;
    pitch_angle            = 0;
    first_barometer_height = 0;
    Pb    = 101325;       // [Pa]
    hb    = 0;            // [m]
    R_as   = 8.31432;     // [N*m/(mol*K)]
    G0     = 9.80665;     // [m/s.^2]
    Lb     = -0.0065;     // [K/m]
    Tb     = 288.15;      // [K]
    M     = 0.0289644;   // [Kg/mol]
    nn = 0, nd = 0, ndiff =0, ndiv=0, d=0, P=0;

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


//    % Predict
//      x_k1k=F*x_kk;
//      P_k1k=F*P_kk*F'+T;

//      % Update
//      % Op 1
//  %     z_est_k(1,1)=(x_kk(1)- x_kk(6))/cos(x_kk(4));
//  %     z_est_k(2,1)=x_kk(3);
//  %     z_est_k(3,1)=x_kk(5);
//  %
//      % Op 2
//  %     z_est_k(1,1)=(x_kk(1)- x_kk(6))/cos(x_kk(4));
//  %     z_est_k(2,1)=x_kk(3);
//  %     z_est_k(3,1)=x_kk(5);
//  %     z_est_k(4,1)=x_kk(2);

//       % Op 3
//      z_est_k(1,1)=(x_kk(1)- x_kk(6))/cos(x_kk(4));
//      z_est_k(2,1)=x_kk(3);
//      z_est_k(3,1)=x_kk(5);
//      z_est_k(4,1)=x_kk(1) + x_kk(7);
//      z_est_k(5,1)=x_kk(4);
//      v=meas(i,:)'-z_est_k;

//       %Innovation (or residual) covariance
//       S=H*P_k1k*H'+R;

//       dis_mahala=v'*S*v;
//   %     if(dis_mahala > 1e3)
//   %         % Store
//   %         output(i,:)=x_k1k';
//   %         % Next iteration
//   %         x_kk=x_k1k;
//   %         P_kk=P_k1k;
//   %         continue;
//   %     end

//       %Near-optimal Kalman gain
//       K=P_k1k*H'*inv(S);

//       %Updated state estimate
//       x_k1k1=x_k1k+K*v;

//       %Updated covariance estimate
//       P_k1k1=(eye(dim_state,dim_state)-K*H)*P_k1k;

//       % Store to display
//       output(i,:)=x_k1k1';

//       % Next iteration
//       x_kk=x_k1k1;
//       P_kk=P_k1k1;

    //Prediction stage
    x_k1k           = F*x_kk;
    p_k1k           = F*p_kk*(F.transpose().eval())+ T;

    //Update stage
    z_est_k(0,0)=  (x_kk(0,0)- x_kk(5,0))/cos(x_kk(3,0));
    z_est_k(1,0)=  x_kk(2,0);
    z_est_k(2,0)=  x_kk(4,0);
    z_est_k(3,0)=  x_kk(0,0) + x_kk(6,0);
    z_est_k(4,0)=  x_kk(3,0);

    //      cout << "z_est_k(0,0) " << z_est_k(0,0) << endl;
    //      cout << "z_est_k(1,0) " << z_est_k(1,0) << endl;
    //      cout << "z_est_k(2,0) " << z_est_k(2,0) << endl;
    //      cout << "z_est_k(3,0) " << z_est_k(3,0) << endl;
    //      cout << "z_est_k(4,0) " << z_est_k(4,0) << endl;

    //Filling in the measurement model
    H(0,0) = 1.0/cos(x_kk(3,0));
    H(0,3) = (x_kk(0,0) - x_kk(5,0))*sin(x_kk(3,0))/cos(x_kk(3,0));
    H(0,5) = -1.0/cos(x_kk(3.0));
    H(1,2) = 1.0;
    H(2,4) = 1.0;
    H(3,0) = 1.0;
    H(3,6) = 1.0;
    H(4,3) = 1.0;



    //Measurement residual
    v(0,0) = measuredAltitude      - z_est_k(0,0);
    v(1,0) = linear_acceleration_z - z_est_k(1,0);
    v(2,0) = angular_velocity      - z_est_k(2,0);
    v(3,0) = barometer_height      - z_est_k(3,0);
    v(4,0) = pitch_angle           - z_est_k(4,0);

    //     cout << "diff Acc " << v(1,0) << endl;

    //Innovation (or residual) covariance
    S=H*p_k1k*(H.transpose().eval()) + R;

  	  
    // dis_mahala=(v.transpose().eval())*S*v;
   //  if(dis_mahala(0,0) > 1e3){
    //    x_kk=x_k1k;
   //     p_kk=p_k1k;
	//		}



    //Near-optimal Kalman gain
    K=p_k1k*(H.transpose().eval())*(S.inverse());

    //Updated state estimate
    x_k1k1=x_k1k+K*v;

    //Updated covariance estimate
    p_k1k1=((I.setIdentity(7,7))-K*H)*p_k1k;

    //Next iteration
    x_kk=x_k1k1;
    p_kk=p_k1k1;

    cout << "x_kk(0,0) " << x_kk << endl;

    altitudeData.header.stamp       = ros::Time::now();
    altitudeData.pose.position.z    = x_kk(0,0);
    if(altitudeData.pose.position.z < 0) altitudeData.pose.position.z = 0;
    droneAltitudePub.publish(altitudeData);

    return true;
}

void DroneAltitudeFiltering::OpenModel()
{
    //Filling the process model x_kk
    x_kk(0,0) = 0;
    x_kk(1,0) = 0;
    x_kk(2,0) = 0;
    x_kk(3,0) = 0;
    x_kk(4,0) = 0;
    x_kk(5,0) = 0;
    x_kk(6,0) = 0;

    //Filling the predicted covariance estimate p_kk
    p_kk(6,6) = 10;

    //Filling in the process covariance Q
    T(0,0) = 0;
    T(1,1) = 0;
    T(2,2) = 0.01;
    T(3,3) = 0.01;
    T(4,4) = 0.01;
    T(5,5) = 10;
    T(6,6) = 0;

    //Filling in the process jacobian
    F(0,0) = 1.0;
    F(0,1) = 0.01;
    F(0,2) = 0.5*pow(0.01,2);
    F(1,1) = 1.0;
    F(1,2) = 0.01;
    F(2,2) = 1.0;
    F(3,3) = 1.0;
    F(3,4) = 0.01;
    F(4,4) = 1.0;
    F(5,5) = 1.0;
    F(6,6) = 1.0;


    //  Filling in the measurement covariance
    R(0,0) = 0.01;    							// altitude by lidar
    R(1,1) = 0.1;									  // accelerations by the imu
    R(2,2) = 10*(M_PI/180);					// angular velocity by imu	
    R(3,3) = 0.8;										// alitude by barometer
    R(4,4) = 1.0;										// pitch angle 

    return;
}

void DroneAltitudeFiltering::droneLidarCallbackSim( const geometry_msgs::PoseStamped &msg)
{

     measuredAltitude = msg.pose.position.z;

     if (abs(measuredAltitude - lastMeasuredAltitude)>altitude_treshold)
     {
         object_height = object_height + (lastMeasuredAltitude - measuredAltitude);
         cout << "Object height" << object_height << endl;

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

    altitudeData.pose.position.z = filteredAltitude;
    //PublishAltitudeData(altitudeData);


    lastMeasuredAltitude=msg.pose.position.z;

}

void DroneAltitudeFiltering::droneLidarCallbackReal(const sensor_msgs::Range &msg)
{

    measuredAltitude = msg.range;

    if (abs(measuredAltitude - lastMeasuredAltitude)>altitude_treshold)
    {
        object_height = object_height + (lastMeasuredAltitude - measuredAltitude);
        cout << "Object height" << object_height << endl;

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

   altitudeData.pose.position.z = filteredAltitude;
   //PublishAltitudeData(altitudeData);


   lastMeasuredAltitude= msg.range;
}

void DroneAltitudeFiltering::droneImuCallback(const sensor_msgs::Imu &msg)
{

    angular_velocity        = msg.angular_velocity.y;
    linear_acceleration_z   = msg.linear_acceleration.z;

    //converting to radians
    angular_velocity = angular_velocity * (M_PI/180);
    //    cout << "angular_velocity" << angular_velocity << endl;


    //removing the bias from the accelerations
    linear_acceleration_z   = linear_acceleration_z - 9.6073;
    //    cout << "linear_acceleration_z" << linear_acceleration_z << endl;


    return;
}

void DroneAltitudeFiltering::droneRotationAnglesCallback(const geometry_msgs::Vector3Stamped &msg)
{
    pitch_angle = msg.vector.y;
    //converting to radians
    pitch_angle = pitch_angle * (M_PI/180);

    //     cout << "pitch_angle" << pitch_angle << endl;

    return;
}

void DroneAltitudeFiltering::droneAtmPressureCallback(const sensor_msgs::FluidPressure &msg)
{

    atm_pressure = msg.fluid_pressure;

    //    Pb = 101325;    % [Pa]
    //       hb = 0;         % [m]
    //       R_as = 8.31432; % [N*m/(mol*K)]
    //       G0 = 9.80665;   % [m/s.^2]
    //       Lb= -0.0065;    % [K/m]
    //       Tb = 288.15;    % [K]
    //       M = 0.0289644;  % [Kg/mol]

    //       P = actualPressure;

    //       nn = Tb * Pb^((R_as * Lb)/(G0 * M));
    //       nd = P.^((R_as * Lb)/(G0 * M));
    //       n = nn ./ nd - Tb;
    //       d = Lb;

    //       h = n / d + hb;

    //       BarometerHeight = h;
    //       BarometerHeight = BarometerHeight - BarometerHeight(1);
    //       BarometerHeight(end) = BarometerHeight(end-1);

    P = atm_pressure;

    nn       = Tb * pow((Pb),((R_as * Lb)/(G0 * M)));
    nd       = pow((P),((R_as * Lb)/(G0 * M)));
    //       cout << "nd " << nd << endl;
    //       cout << "ndiff " << ndiff << endl;
    ndiv     = nn / nd - Tb;
    //       cout << "ndiv " << ndiv << endl;
    d = Lb;

    if(counter == 0){
        first_barometer_height = ndiv / d + hb;
        counter++;
    }

    barometer_height = ndiv / d + hb;

    barometer_height = barometer_height - first_barometer_height;

    //        cout << "barometer_height" << barometer_height << endl;


}

void DroneAltitudeFiltering::droneTemperatureCallback(const sensor_msgs::Temperature &msg)
{
    temperature = msg.temperature;
    //    cout << "temperature" << temperature << endl;
    return;
}

void DroneAltitudeFiltering::PublishAltitudeData(const geometry_msgs::PoseStamped &altitudemsg)
{
    if(moduleStarted == true)
        droneAltitudePub.publish(altitudemsg);
    return;
}
