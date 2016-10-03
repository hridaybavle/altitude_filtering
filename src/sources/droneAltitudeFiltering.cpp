/*
 *  Created on: 22.06.2016.
 *      Author: Zorana Milosevic
 */


#include "droneAltitudeFiltering.h"

#include <iostream>
#include <fstream>

// using namespaces only in cpp files!!
using namespace std;

Eigen::MatrixXd x_kk(8,1),p_kk(8,8), T(8,8),  F(8,8);
Eigen::MatrixXd H(6,8), R(6,6);
Eigen::MatrixXd x_k1k(8,1), p_k1k(8,8);
Eigen::MatrixXd z_est_k(6,1);
Eigen::MatrixXd v(6,1), S(6,6);
Eigen::MatrixXd dis_mahala(6,1);
Eigen::MatrixXd K(8,6);
Eigen::MatrixXd x_k1k1(8,1), p_k1k1(8,8);
Eigen::MatrixXd I(8,8);


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

    droneLidarSim            = n.subscribe("/px4flow/raw/px4Flow_pose_z", 1, &DroneAltitudeFiltering::droneLidarCallbackSim, this);
    droneLidarReal           = n.subscribe("lightware_altitude",1, &DroneAltitudeFiltering::droneLidarCallbackReal, this);
    droneImuSub              = n.subscribe("mavros/imu/data",1,&DroneAltitudeFiltering::droneImuCallback, this);
    droneRotationAnglesSub   = n.subscribe("rotation_angles",1, &DroneAltitudeFiltering::droneRotationAnglesCallback, this);
    droneAtmPressureSub      = n.subscribe("mavros/imu/atm_pressure",1,&DroneAltitudeFiltering::droneAtmPressureCallback, this);
    droneTemperatureSub      = n.subscribe("mavros/imu/temperature",1,&DroneAltitudeFiltering::droneTemperatureCallback, this);
    droneMavrosAltitudeSub   = n.subscribe("mavros/altitude",1, &DroneAltitudeFiltering::droneMavrosAltitudeCallback, this);
    droneStatusSub           = n.subscribe("status",1, &DroneAltitudeFiltering::droneStatusCallback, this);

    droneAltitudePub         = n.advertise<geometry_msgs::PoseStamped>("altitudeFiltered", 1, true);
    droneBarometerHeightPub  = n.advertise<geometry_msgs::PoseStamped>("altitudeBarometer",1, true);
    droneEstObjectHeightPub  = n.advertise<geometry_msgs::PoseStamped>("objectHeightEstimated",1,true);
    droneObjectHeightPub     = n.advertise<geometry_msgs::PoseStamped>("objectHeight",1, true);
		droneAccelerationsPub		 = n.advertise<geometry_msgs::PoseStamped>("accelerations",1,true);
	
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
		count 			 			 = 0;
		stop_count				 = 0;
	  peak_counter 			 =-1;
		object_counter 		 = 0;		
    timeNow 					 = 0;
		x_kk.setZero(8,1),p_kk.setZero(8,8), T.setZero(8,8),  F.setZero(8,8);
    H.setZero(6,8), R.setZero(6,6);
    x_k1k.setZero(8,1), p_k1k.setZero(8,8);
    z_est_k.setZero(6,1);
    v.setZero(6,1), S.setZero(6,6);
    K.setZero(8,6);
    x_k1k1.setZero(8,1), p_k1k1.setZero(8,8);
    I.setZero(8,8);


    measuredAltitude       = 0;
    linear_acceleration_z  = 0;
    avg_linear_acceleration_z = 0;
    angular_velocity       = 0;
    barometer_height       = 0;
    pitch_angle            = 0;
    first_barometer_height = 0;
    first_measured_lidar_altitude = 0;
    //Pb     = 101325;       // [Pa]
    //hb     = 0;            // [m]
    //R_as   = 8.31432;     // [N*m/(mol*K)]
    //G0     = 9.80665;     // [m/s.^2]
    //Lb     = -0.0065;     // [K/m]
    //Tb     = 288.15;      // [K]
    //M     = 0.0289644;   // [Kg/mol]
    //nn = 0, nd = 0, ndiff =0, ndiv=0, d=0, P=0;

    return true;
}


void DroneAltitudeFiltering::close()
{
    DroneModule::close();
    return;
}

bool DroneAltitudeFiltering::resetValues()
{
    if(!DroneModule::resetValues())
        return false;

    counter = 0;
		object_counter = 0;

    return true;

}

bool DroneAltitudeFiltering::startVal()
{
    if(!DroneModule::startVal())
        return false;

    counter = 0;
		object_counter = 0;

    return true;

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

    //Filling in the process jacobian
    F(0,0) = 1.0;
    F(0,1) = 1.0*deltaT;
    F(0,2) = 0.5*pow(deltaT,2);
    F(1,1) = 1.0;
    F(1,2) = 1.0*deltaT;
    F(2,2) = 1.0;
    F(3,3) = 1.0;
    F(3,4) = 1.0*deltaT;
    F(4,4) = 1.0;
    F(5,5) = 1.0;
    F(6,6) = 1.0;
    F(7,7) = 1.0;

    //Prediction stage
    x_k1k           = F*x_kk;
    p_k1k           = F*p_kk*(F.transpose().eval())+ T;

    //Update stage
    z_est_k(0,0)=  (x_kk(0,0)- x_kk(5,0))/cos(x_kk(3,0));
    z_est_k(1,0)=  x_kk(2,0) + x_kk(7,0);
    z_est_k(2,0)=  x_kk(4,0);
    z_est_k(3,0)=  x_kk(0,0) + x_kk(6,0);
    z_est_k(4,0)=  x_kk(3,0);
		z_est_k(5,0)=  x_kk(5,0);
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
    H(1,7) = 1.0;
    H(2,4) = 1.0;
    H(3,0) = 1.0;
    H(3,6) = 1.0;
    H(4,3) = 1.0;
    H(5,5) = 1.0;



    //Measurement residual
    v(0,0) = measuredAltitude      - z_est_k(0,0);
    v(1,0) = linear_acceleration_z - z_est_k(1,0);
    v(2,0) = angular_velocity      - z_est_k(2,0);
    v(3,0) = barometer_height      - z_est_k(3,0);
    v(4,0) = pitch_angle           - z_est_k(4,0);
		v(5,0) = object_height 				 - z_est_k(5,0);
    //     cout << "diff Acc " << v(1,0) << endl;

    //Innovation (or residual) covariance
    S=H*p_k1k*(H.transpose().eval()) + R;


    //Near-optimal Kalman gain
    K=p_k1k*(H.transpose().eval())*(S.inverse());

    //Updated state estimate
    x_k1k1=x_k1k+K*v;

    //Updated covariance estimate
    p_k1k1=((I.setIdentity(8,8))-K*H)*p_k1k;

    //Next iteration
    x_kk=x_k1k1;
    p_kk=p_k1k1;

    // if the altitude overshoots
    //dis_mahala =(v.transpose().eval())*S*v;
    //if(dis_mahala(0,0) > 1e1){
    //	cout << "entered maha dist"  << endl;
    //  x_kk=x_k1k;
    //  p_kk=p_k1k;
    // }

    //cout << "Dist maha" << dis_mahala << endl;
    //cout << "x_kk(5,0) " << x_kk(5,0) << endl;

    altitudeData.header.stamp       = ros::Time::now();
    altitudeData.pose.position.z    = x_kk(0,0);
    if(altitudeData.pose.position.z < 0) altitudeData.pose.position.z = 0;

    droneAltitudePub.publish(altitudeData);

    objectHeightEstData.header.stamp = ros::Time::now();
    objectHeightEstData.pose.position.z = x_kk(5,0);

    droneEstObjectHeightPub.publish(objectHeightEstData);

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
    x_kk(7,0) = 0;

    //Filling the predicted covariance estimate p_kk
    p_kk(6,6) = 10;
    p_kk(7,7) = 10;

    //Filling in the process covariance Q
    T(0,0) = 0;
    T(1,1) = 0;
    T(2,2) = 0.01;
    T(3,3) = 0.01;
    T(4,4) = 0.01;
    T(5,5) = 10;
    T(6,6) = 0;
    T(7,7) = 0;


    //Filling in the process jacobian (initializing with deltaT = 0.01)
    F(0,0) = 1.0;
    F(0,1) = 1.0*0.01;
    F(0,2) = 0.5*pow(0.01,2);
    F(1,1) = 1.0;
    F(1,2) = 1.0*0.01;
    F(2,2) = 1.0;
    F(3,3) = 1.0;
    F(3,4) = 1.0*0.01;
    F(4,4) = 1.0;
    F(5,5) = 1.0;
    F(6,6) = 1.0;
    F(7,7) = 1.0;


    //  Filling in the measurement covariance
    R(0,0) = 1.0;                           // altitude by lidar
    R(1,1) = 1.0;							// accelerations by the imu
    R(2,2) = 10*(M_PI/180);                // angular velocity by imu
    R(3,3) = 10.0;                          // alitude by barometer
    R(4,4) = 0.1;						   // pitch angle
		R(5,5) = 5.0;              //object height
	
    return;
}

void DroneAltitudeFiltering::droneLidarCallbackSim( const geometry_msgs::PoseStamped &msg)
{

    measuredAltitude = msg.pose.position.z;

    //     if (abs(measuredAltitude - lastMeasuredAltitude)>altitude_treshold)
    //     {
    //         object_height = object_height + (lastMeasuredAltitude - measuredAltitude);
    //         cout << "Object height" << object_height << endl;

    //         if(abs(object_height) < 0.05){
    //            object_below = false;
    //         }
    //         else {
    //            object_below = true;
    //         }

    //     }

    //     if(object_below){
    //         filteredAltitude = measuredAltitude + object_height;
    //     }
    //     else {
    //         filteredAltitude = measuredAltitude;
    //     }

    //    altitudeData.pose.position.z = filteredAltitude;
    //    //PublishAltitudeData(altitudeData);


    lastMeasuredAltitude=msg.pose.position.z;

}

void DroneAltitudeFiltering::droneLidarCallbackReal(const sensor_msgs::Range &msg)
{

    measuredAltitude = msg.range;

    if(measuredAltitude > 1.5 && x_kk(5,0) <= 0)
    {
        counter = 0;
        //cout << "Resetting the Altitude Filter " << endl;
    }

    //if(x_kk(5,0) < 0 && x_kk(0,0) - measuredAltitude > 0.20)
    //{
    //	counter = 0;
    //	cout << "Resetting the Altitude Filter " << endl;
    //}

    /// Store the values in a buffer
    if(lidar_measurements.size() < BUFFER_SIZE){
        lidar_measurements.push_back(measuredAltitude);
    }
    else{
        double mean = 0;
        // Shift the vector
        for(int i = 0; i < lidar_measurements.size() - 1; i++){
            lidar_measurements[i] = lidar_measurements[i + 1];

            //adding the lidarmeasurements
            mean += lidar_measurements[i+1];
        }
        lidar_measurements[lidar_measurements.size() - 1] = measuredAltitude;

        //computing the average of the remaning samples
        mean /= (lidar_measurements.size() - 1);
				
				// If peak analyzer enabled, decrease peak counter
				if(peak_counter > 0) peak_counter--;

        if((abs(lidar_measurements[lidar_measurements.size() - 1] - mean) > (double) ALTITUDE_THRESHOLD) 
																																										&& (peak_counter == -1)){
						prev_mean = mean;
						peak_counter = BUFFER_SIZE - 1;
				}
				
				if (peak_counter == 0){
					std::vector<double> lidar_sorted = lidar_measurements;
					std::sort(lidar_sorted.begin(), lidar_sorted.end());
					object_height -= (lidar_sorted[(int) ( ( BUFFER_SIZE - 1 ) / 2 )] - prev_mean);
					peak_counter = -1;				
				}

				// No object can be negative
				if (object_height < 0)	object_height = 0;
			
				//TODO : Debug more errors
				if (object_height < OBJECT_THRESHOLD)	object_height = 0;
	
				if(object_counter == 0){
					 object_height = 0;
					 object_counter++;
				}							

        // publish object_height
        objectHeightData.header.stamp    = ros::Time::now();
        objectHeightData.pose.position.z = object_height;
        droneObjectHeightPub.publish(objectHeightData);
    }





    //    if (abs(measuredAltitude - lastMeasuredAltitude)>altitude_treshold)
    //    {
    //        object_height = object_height + (lastMeasuredAltitude - measuredAltitude);
    //        cout << "Object height" << object_height << endl;

    //        if(abs(object_height) < 0.05){
    //           object_below = false;
    //        }
    //        else {
    //           object_below = true;
    //        }

    //    }

    //    if(object_below){
    //        filteredAltitude = measuredAltitude + object_height;
    //    }
    //    else {
    //        filteredAltitude = measuredAltitude;
    //    }

    //  altitudeData.pose.position.z = filteredAltitude;
    //PublishAltitudeData(altitudeData);


    //    lastMeasuredAltitude= msg.range;
}

void DroneAltitudeFiltering::droneImuCallback(const sensor_msgs::Imu &msg)
{

    timePrev = timeNow;
    timeNow = (double) ros::Time::now().sec + ((double) ros::Time::now().nsec / (double) 1E9);

    deltaT   = timeNow - timePrev;

    angular_velocity        = msg.angular_velocity.y;
    linear_acceleration_z   = msg.linear_acceleration.z;

    //converting to radians
    angular_velocity = angular_velocity * (M_PI/180);
    //cout << "angular_velocity" << angular_velocity << endl;

    if(count < ACCELERATIONS_COUNT){
        avg_linear_acceleration_z += msg.linear_acceleration.z;
				linear_acceleration_z   = linear_acceleration_z - 9.8;
        count++;
    }
    else if(stop_count == 0){
        avg_linear_acceleration_z /= (float) ACCELERATIONS_COUNT;
        cout << "avg_linear_acceleration_z" << avg_linear_acceleration_z << endl;
				linear_acceleration_z   = linear_acceleration_z - avg_linear_acceleration_z;
        stop_count++;
    }
		else{
				//removing the bias from the accelerations
        linear_acceleration_z = linear_acceleration_z - avg_linear_acceleration_z;
    }
      
		run();
    cout << "linear_acceleration_z" << linear_acceleration_z << endl;
		// publish object_height
    accelerationData.header.stamp    = ros::Time::now();
    accelerationData.pose.position.z = linear_acceleration_z;
    droneAccelerationsPub.publish(accelerationData);

    return;
}

void DroneAltitudeFiltering::droneRotationAnglesCallback(const geometry_msgs::Vector3Stamped &msg)
{
    pitch_angle = msg.vector.y;
    //converting to radians
    pitch_angle = pitch_angle * (M_PI/180);

    //cout << "pitch_angle" << pitch_angle << endl;

    return;
}

void DroneAltitudeFiltering::droneAtmPressureCallback(const sensor_msgs::FluidPressure &msg)
{

    atm_pressure = msg.fluid_pressure;

    //       Pb = 101325;    % [Pa]
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

//    P = atm_pressure;

//    nn       = Tb * pow((Pb),((R_as * Lb)/(G0 * M)));
//    nd       = pow((P),((R_as * Lb)/(G0 * M)));
//    ndiv     = nn / nd - Tb;
//    d = Lb;

//		if(!(droneStatus.status == droneMsgsROS::droneStatus::FLYING))
//    {
//        barometer_height = measuredAltitude;
//    }
//		else{
//        if(counter == 0){
//            first_barometer_height = (ndiv / d + hb);
//						first_measured_lidar_altitude = measuredAltitude;
//            counter++;
//        }

//				barometer_height = first_measured_lidar_altitude + ((ndiv / d + hb) - first_barometer_height);
//		}

//		barometerData.header.stamp       	= ros::Time::now();
//   	barometerData.pose.position.z 		= barometer_height;
//    droneBarometerHeightPub.publish(barometerData);
		//cout << "barometer_height" << barometer_height << endl;


}

void DroneAltitudeFiltering::droneTemperatureCallback(const sensor_msgs::Temperature &msg)
{
    temperature = msg.temperature;
    //cout << "temperature" << temperature << endl;
    return;
}

void DroneAltitudeFiltering::droneMavrosAltitudeCallback(const mavros_msgs::Altitude &msg)
{

    if(!(droneStatus.status == droneMsgsROS::droneStatus::FLYING))
    {
        barometer_height = measuredAltitude;
    }

    else
    {

        if(counter == 0){
            first_barometer_height        = msg.local;
            first_measured_lidar_altitude = measuredAltitude;
            counter++;
        }

        //cout << "first barometer height " << first_barometer_height << endl;
        //cout << "first lidar height"      << first_measured_lidar_altitude << endl;
        //cout << "barometer_height - first_barometer_height " << msg.local - first_barometer_height << endl;
        barometer_height = first_measured_lidar_altitude + (msg.local - first_barometer_height);
    }

    barometerData.header.stamp       	= ros::Time::now();
    barometerData.pose.position.z 		= barometer_height;
    droneBarometerHeightPub.publish(barometerData);

    //cout << "barometer_height" << barometer_height << endl;

}


void DroneAltitudeFiltering::droneStatusCallback(const droneMsgsROS::droneStatus &msg)
{
    droneStatus = msg;
    return;
}

void DroneAltitudeFiltering::PublishAltitudeData(const geometry_msgs::PoseStamped &altitudemsg)
{
    if(moduleStarted == true)
        droneAltitudePub.publish(altitudemsg);
    return;
}
