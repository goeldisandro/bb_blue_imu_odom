
extern "C" {
#include "rc_usefulincludes.h"
#include "roboticscape.h"
}

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
//#include "geometry_msgs/Quaternion.h"


int main(int argc, char *argv[]){
	ros::init(argc, argv, "imu_node");
	ros::NodeHandle nh;
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 50);
	ros::Rate r(200); // 200 hz

	sensor_msgs::Imu imu;

//  	imu.linear_acceleration_covariance[0] = linear_acceleration_stddev;
//  	imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
//  	imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

//  	imu.angular_velocity_covariance[0] = angular_velocity_stddev;
//  	imu.angular_velocity_covariance[4] = angular_velocity_stddev;
//  	imu.angular_velocity_covariance[8] = angular_velocity_stddev;

//  	imu.orientation_covariance[0] = orientation_stddev;
//  	imu.orientation_covariance[4] = orientation_stddev;
//  	imu.orientation_covariance[8] = orientation_stddev;	
	

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
		
		// publish imu message
	        imu.header.stamp = measurement_time;
	        imu.header.frame_id = "imu_link";

		if(rc_read_gyro_data(&data)<0){
			printf("read gyro data failed\n");
		}

		imu.angular_velocity.x = (double)(data.gyro[0]*DEG_TO_RAD);
		imu.angular_velocity.y = (double)(data.gyro[1]*DEG_TO_RAD);
		imu.angular_velocity.z = (double)(data.gyro[2]*DEG_TO_RAD);

		if(rc_read_accel_data(&data)<0){
			printf("read accel data failed\n");
		}

        	imu.linear_acceleration.x = (double)(data.accel[0]);
        	imu.linear_acceleration.y = (double)(data.accel[1]);
        	imu.linear_acceleration.z = (double)(data.accel[2]);

//		printf("%6.2f %6.2f %6.2f |",	data.accel[0],\
//						data.accel[1],\
//						data.accel[2]);
//		fflush(stdout);

        	imu_pub.publish(imu);

		rc_usleep(10000);
	}

	rc_power_off_imu();
	rc_cleanup();
	return 0;
}
