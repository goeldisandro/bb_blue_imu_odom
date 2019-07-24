extern "C"{
#include "rc_usefulincludes.h"
#include "roboticscape.h"
}
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/Imu.h"
#include "std_srvs/Empty.h"
//#include "geometry_msgs/Quaternion.h"

bool zero_orientation_set = false;

bool set_odom_zero_orientation(std_srvs::Empty::Request&, std_srvs::Empty::Response&){
  	zero_orientation_set = false;
  	return true;
}


int main(int argc, char *argv[]){
	// Enc Values
	int encR;
	int encL;
	// Enc Values old
	int prevEncR = 0;
	int prevEncL = 0;

	double U = 0.633;   	// circumference of a wheel
	double D = 0.558;	// distance between wheels
	double k = 1.00; 		// korrekturfaktro >1 nach Rechts 
	// x and y translation and angle of the robot
	double x = 0.0;
	double y = 0.0;
	double theta = 0;

	

	ros::init(argc, argv, "imu_node");
	ros::NodeHandle nh;
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 50);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
	ros::ServiceServer service = nh.advertiseService("set_odom_zero_orientation", set_odom_zero_orientation);

//	ros::Rate r(200); // 200 hz

	sensor_msgs::Imu imu;
	nav_msgs::Odometry odom_msg;

	rc_imu_data_t data; //struct to hold new data
	
		opterr = 0;

	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}

	// use defaults for now, except also enable magnetometer.
	rc_imu_config_t conf = rc_default_imu_config();
	conf.enable_magnetometer=1;
	

	if(rc_initialize_imu(&data, conf)){
		fprintf(stderr,"rc_initialize_imu_failed\n");
		return -1;
	}

	//now just wait, print_data will run
	while (rc_get_state() != EXITING) {
		printf("\r");

		// calculate measurement time
		ros::Time measurement_time = ros::Time::now();

		// read IMU-Data
		if(rc_read_gyro_data(&data)<0){
			printf("read gyro data failed\n");
		}
		
		// calculating odom
		encR = rc_get_encoder_pos(1);
		encL = -rc_get_encoder_pos(2);	

		if (!zero_orientation_set){
			prevEncR = encR;
			prevEncL = encL;			
			x = 0.0;
	  		y = 0.0;
			theta = 0;
			zero_orientation_set = true;
		}

		double dEr = (encR-prevEncR)/k;
		double dEl = (encL-prevEncL)*k;	

		theta += U*(dEr-dEl)/(D*2000);      // angle of the robot, 2000 are the ticks per wheel rotation	
		if(theta > 6.283) theta=theta-6.283;
		if(theta < 0) theta=theta+6.283;
		x += U/2000*(dEr+dEl)/2*cos(theta); //calculate x value
		y += U/2000*(dEr+dEl)/2*sin(theta); //calculate y value

		prevEncR = encR;
		prevEncL = encL;

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

		// publish odom message
		odom_msg.header.stamp = measurement_time;
		odom_msg.header.frame_id = "odom";
		odom_msg.child_frame_id = "base_link";

		odom_msg.pose.pose.position.x = x;
		odom_msg.pose.pose.position.y = y;
		odom_msg.pose.pose.position.z = 0.0;
		odom_msg.pose.pose.orientation = odom_quat;

		odom_msg.twist.twist.linear.x = 0;
		odom_msg.twist.twist.linear.y = 0;
		odom_msg.twist.twist.angular.z = 0;

		odom_pub.publish(odom_msg);


	        // publish imu message
	        imu.header.stamp = measurement_time;
	        imu.header.frame_id = "imu_link";

		imu.angular_velocity.x = (double)(data.gyro[0]*DEG_TO_RAD);
		imu.angular_velocity.y = (double)(data.gyro[1]*DEG_TO_RAD);
		imu.angular_velocity.z = (double)(data.gyro[2]*DEG_TO_RAD);

		if(rc_read_accel_data(&data)<0){
			printf("read accel data failed\n");
		}

        	imu.linear_acceleration.x = (double)(data.accel[0]);
        	imu.linear_acceleration.y = (double)(data.accel[1]);
        	imu.linear_acceleration.z = (double)(data.accel[2]);

        	imu_pub.publish(imu);

		rc_usleep(10000);
		ros::spinOnce();
	}

	rc_power_off_imu();
	rc_cleanup();
	return 0;
}


