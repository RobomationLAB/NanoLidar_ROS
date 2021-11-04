#include "serial.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nanolidar");
    ros::NodeHandle nh("~");
    
    nanolidar lidar;
    nh.param<int>("baudrate", lidar.baudrate, 115200);
    nh.param<std::string>("port", lidar.port, "/dev/ttyUSB0");
    nh.param<std::string>("frame_id", lidar.frame_id, "laser_frame");
    nh.param<std::string>("scan_topic", lidar.scan_topic, "/scan");
    nh.param<double>("angle_min", lidar.angle_min , 0.0);
	nh.param<double>("angle_max", lidar.angle_max , 360.0);
	nh.param<double>("angle_increment", lidar.angle_increment , 1.0);    
    nh.param<double>("range_min", lidar.range_min, 0.005);
	nh.param<double>("range_max", lidar.range_max, 3.0);

    lidar.connect();
    lidar.NanoLidarCMD = nh.subscribe("/nanolidarCMD", 10, &nanolidar::CMD_Callback, &lidar);
    lidar.scan_pub = nh.advertise<sensor_msgs::LaserScan>(lidar.scan_topic.c_str(), 1);	

    ros::spin();

    return 0;
}