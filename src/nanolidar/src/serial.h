
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <termios.h>
#include <fcntl.h>
#include <thread>
#include <mutex>


#include "ros/ros.h"
#include "std_msgs/String.h"

class nanolidar
{
private:

	std::mutex fd_mtx;	
	int fd;
	std::thread receive_thread;
	unsigned char index = 0x00;
	double lidarscan[360];
	double lidarIntensity[360];

	
	void send_task(void);
	void receive_task(void);
	
public:
	bool connect(void);
	bool start(void);
	bool stop(void);
	void speckle_filter(void);

	ros::Subscriber NanoLidarCMD;
	ros::Publisher scan_pub;

	void CMD_Callback(const std_msgs::String::ConstPtr &msg);
	void scan_publish(ros::Time start_scan_time, double scan_duration, double* lidarscan);

	int baudrate;
	std::string port;    
    std::string frame_id, scan_topic;
	sensor_msgs::LaserScan scan_msg;
	double angle_max,angle_min;
    double range_max, range_min;
	double angle_increment;
	ros::Time start_scan_time, end_scan_time;
	double scan_duration;
};

bool nanolidar::start(void)
{
	unsigned char send_serial_protocol[6];
	send_serial_protocol[0] = (unsigned char)0xFD;
	send_serial_protocol[1] = (unsigned char)0xE0;
	send_serial_protocol[2] = (unsigned char)0x02;
	send_serial_protocol[3] = (unsigned char)0x0F;

	uint8_t CK_A = 0;
	uint8_t CK_B = 0;
	for (int i = 0; i < 4; i++)
	{
		CK_A = CK_A + send_serial_protocol[i];
		CK_B = CK_B + CK_A;
	}
	send_serial_protocol[4] = (unsigned char)CK_A;
	send_serial_protocol[5] = (unsigned char)CK_B;
	
	unsigned char received[12];
	memset(received, 0, 12);
	fd_mtx.lock();
	int val = write(fd, send_serial_protocol, sizeof(send_serial_protocol));
	// printf("start------send_serial_protocol: ");
    // for(int i=0; i<val; i++)
	// 	printf("%d, ", (int)send_serial_protocol[i]);
	// printf("\n");
	fd_mtx.unlock();	
}
bool nanolidar::stop(void)
{
	unsigned char send_serial_protocol[6];
	send_serial_protocol[0] = (unsigned char)0xFD;
	send_serial_protocol[1] = (unsigned char)0xE0;
	send_serial_protocol[2] = (unsigned char)0x04;
	send_serial_protocol[3] = (unsigned char)0xF0;

	uint8_t CK_A = 0;
	uint8_t CK_B = 0;
	for (int i = 0; i < 4; i++)
	{
		CK_A = CK_A + send_serial_protocol[i];
		CK_B = CK_B + CK_A;
	}
	send_serial_protocol[4] = (unsigned char)CK_A;
	send_serial_protocol[5] = (unsigned char)CK_B;
	
	unsigned char received[12];
	memset(received, 0, 12);
	fd_mtx.lock();	
	int val = write(fd, send_serial_protocol, sizeof(send_serial_protocol));
	// printf("stop------send_serial_protocol: ");
	
    // for(int i=0; i<val; i++)
	// 	printf("%d, ", send_serial_protocol[i]);
	// printf("\n");	
	fd_mtx.unlock();	
}
void nanolidar::speckle_filter(void)
{
	float p0, p1 ,p2;	

	for (int i = 0; i < 360; i++)
	{
		if (i == 0)
		{
			p0 = lidarscan[359];
			p1 = lidarscan[0];
			p2 = lidarscan[1];
		}else if (i == 359)
		{
			p0 = lidarscan[358];
			p1 = lidarscan[359];
			p2 = lidarscan[0];
		}
		else{
			p0 = lidarscan[i-1];
			p1 = lidarscan[i];
			p2 = lidarscan[i+1];
		}

		float dist1 = sqrt((p0-p1)*(p0-p1));
		float dist2 = sqrt((p2-p1)*(p2-p1));
		// std::cout <<  "p0: " << p0 << std::endl;
		// std::cout <<  "p1: " << p1 << std::endl;
		// std::cout <<  "p2: " << p2 << std::endl;

		// std::cout <<  "i: " << i <<" dist1: " << dist1 << "  dist2: " << dist2 << std::endl;
		if( dist1 < 0.1 || dist2 < 0.1)
		{
			lidarscan[i] = lidarscan[i];
		}
		else
		{
			lidarscan[i] = std::numeric_limits<float>::infinity();
		}
		// std::cout <<  "i: " << i <<" lidarscan: " << lidarscan[i] << std::endl;
	}
}



void nanolidar::receive_task(void)
{

	unsigned char header[1];
	unsigned char MsgsID[1];
	unsigned char WaitData[4];
	unsigned char Description[5];
	unsigned char Distance[360];
	int iter = 0;
	ros::Rate rate(100);
	while (ros::ok())
	{
		
		int read_size = read(fd, header, 1);
		if (read_size == 1 && header[0] == 0xFE)
		{
			start_scan_time = ros::Time::now();
			fd_mtx.lock();						
			read_size = read(fd, MsgsID, 1);
			if (read_size == 1 && MsgsID[0] == 0x30)
			{	
				read_size = read(fd, WaitData, 4);			
				// printf("Lidar ON and wait cmd------(%d) \n", read_size);
			}
			if (read_size == 1 && MsgsID[0] == 0x10)
			{
				read_size = read(fd, Description, 6);
				// printf("Getting Data Discription------------(%d)\n", read_size);
				// for (int i = 0; i < read_size; i++)
				// 	printf("%d, ", Description[i]);
				// printf("\n");

				int data_read_size = read(fd, Distance, 180);
				// printf("Getting Data -----------data(%d) \n", data_read_size);
				for (int i = 0; i < data_read_size; i = i + 2)
				{					
					int distance = Distance[i+1] + Distance[i] * 256;
					// printf("[%d] %d ", i/2, distance);
					lidarscan[iter++] = distance*0.001;
				}
				// printf("\n");
			}
			fd_mtx.unlock();
			if(Description[2]==4){
				speckle_filter();
				end_scan_time = ros::Time::now();
				scan_duration = (end_scan_time - start_scan_time).toSec();
				scan_publish(start_scan_time, scan_duration, lidarscan);
				iter = 0;
			}		
		}
		else
		{
			continue;
		}
		rate.sleep();
	}
}

void nanolidar::CMD_Callback(const std_msgs::String::ConstPtr &msg)
{
	std::string ss_start, ss_stop;
	ss_start = "start";
	ss_stop = "stop";

	if (msg->data.find(ss_start) == 0)
	{
		std::cout << "START reading data" << std::endl;
		index =  index + 0x02;
		start();
	}
	if (msg->data.find(ss_stop) == 0)
	{
		std::cout << "STOP reading data" << std::endl;
		index =  index + 0x02;
		stop();
	}
}

bool nanolidar::connect(void)
{
	fd = 0;
	struct termios newtio;

	while(ros::ok())
	{
		fd = open(port.c_str(), O_RDWR | O_NOCTTY );
		if(fd<0) 
		{ 
			ROS_ERROR("[NanoLidar] connecion error %d. retry", fd);
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		else break;
	}

	
    memset( &newtio, 0, sizeof(newtio) );

    newtio.c_cflag = B115200;
    ROS_INFO("[NanoLidar] connetion established. baudrate: B115200");
  	newtio.c_cflag |= CS8;      // data length 8bit 
    newtio.c_cflag |= CLOCAL;	// Use iternel commutication port
    newtio.c_cflag |= CREAD;	// enable read & write
    newtio.c_iflag = 0;			// no parity bit
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    tcflush (fd, TCIFLUSH );
    tcsetattr(fd, TCSANOW, &newtio );

	//init Serial field
	//send_thread = std::thread(&nanolidar::send_task, this);
	receive_thread = std::thread(&nanolidar::receive_task, this);
	return true;
}

void nanolidar::scan_publish(ros::Time start_scan_time, double scan_duration, double* lidarscan)
{
	scan_msg.header.frame_id = frame_id;
	scan_msg.header.seq = 1;
	scan_msg.header.stamp = start_scan_time;
	
	scan_msg.scan_time = scan_duration;
	scan_msg.time_increment = scan_duration / (double) (360 -1);

	scan_msg.angle_max = angle_max * M_PI / 180.0;
	scan_msg.angle_min = angle_min * M_PI / 180.0;
	scan_msg.angle_increment = M_PI / 180.0; // 1 degree
	
	scan_msg.range_max = range_max;
	scan_msg.range_min = range_min;
	
	scan_msg.ranges.resize(360);
	scan_msg.intensities.resize(360);
	for(int i = 0; i < 360; i++){
		
		if(lidarscan[359-i] < range_min){
			scan_msg.ranges[i] = -std::numeric_limits<float>::infinity();
		}else if(lidarscan[359-i] > range_max){
			scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
		}else{
			scan_msg.ranges[i] = lidarscan[359-i];
		}

		if(i<90)
			scan_msg.intensities[i] = 90;
		else if(i<180)
			scan_msg.intensities[i] = 180;
		else if(i<270)
			scan_msg.intensities[i] = 270;
		else if(i<360)
			scan_msg.intensities[i] = 360;
	}	

	scan_pub.publish(scan_msg);
}
