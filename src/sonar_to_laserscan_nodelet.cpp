#include <sonar_to_laserscan/sonar_to_laserscan_nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include "fcntl.h"
#include "termios.h"
#include "string.h"
#include <pthread.h>
#include <iostream>
#define R 100
#define FALSE -1  
#define TRUE 0  


int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,B38400, B19200, B9600, B4800, B2400, B1200, B300, };  
int name_arr[] = {38400,  19200,  9600,  4800,  2400,  1200,  300, 38400, 19200,  9600, 4800, 2400, 1200,  300, };  
void set_speed(int fd, int speed){  
	int   i;   
	int   status;   
	struct termios   Opt;  
	tcgetattr(fd, &Opt);   
	for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) {   
		if  (speed == name_arr[i]) {       
			tcflush(fd, TCIOFLUSH);       
			cfsetispeed(&Opt, speed_arr[i]);    
			cfsetospeed(&Opt, speed_arr[i]);     
			status = tcsetattr(fd, TCSANOW, &Opt);    
			if  (status != 0) {          
				perror("tcsetattr fd1");    
				return;       
			}      
			tcflush(fd,TCIOFLUSH);     
		}    
	}  
}  

int set_Parity(int fd,int databits,int stopbits,int parity)  
{   
	struct termios options;   
	if  ( tcgetattr( fd,&options)  !=  0) {   
		perror("SetupSerial 1");       
		return(FALSE);    
	}  
	options.c_cflag &= ~CSIZE;   
	switch (databits)   
	{     
		case 7:       
			options.c_cflag |= CS7;   
			break;  
		case 8:       
			options.c_cflag |= CS8;  
			break;     
		default:      
			fprintf(stderr,"Unsupported data size\n"); return (FALSE);    
	}  
	switch (parity)   
	{     
		case 'n':  
		case 'N':      
			options.c_cflag &= ~PARENB;   /* Clear parity enable */  
			options.c_iflag &= ~INPCK;     /* Enable parity checking */   
			break;    
		case 'o':     
		case 'O':       
			options.c_cflag |= (PARODD | PARENB);   
			options.c_iflag |= INPCK;             /* Disnable parity checking */   
			break;    
		case 'e':    
		case 'E':     
			options.c_cflag |= PARENB;     /* Enable parity */      
			options.c_cflag &= ~PARODD;      
			options.c_iflag |= INPCK;       /* Disnable parity checking */  
			break;  
		case 'S':   
		case 's':  /*as no parity*/     
			options.c_cflag &= ~PARENB;  
			options.c_cflag &= ~CSTOPB;break;    
		default:     
			fprintf(stderr,"Unsupported parity\n");      
			return (FALSE);    
	}    

	switch (stopbits)  
	{     
		case 1:      
			options.c_cflag &= ~CSTOPB;    
			break;    
		case 2:      
			options.c_cflag |= CSTOPB;    
			break;  
		default:      
			fprintf(stderr,"Unsupported stop bits\n");    
			return (FALSE);   
	}   
	/* Set input parity option */   
	if (parity != 'n')     
		options.c_iflag |= INPCK;   
	tcflush(fd,TCIFLUSH);  
	options.c_cc[VTIME] = 150;   
	options.c_cc[VMIN] = 0; /* Update the options and do it NOW */  
	if (tcsetattr(fd,TCSANOW,&options) != 0)     
	{   
		perror("SetupSerial 3");     
		return (FALSE);    
	}   
	return (TRUE);    
} 

namespace sonar_to_laserscan
{

	SonarToLaserScanNodelet::SonarToLaserScanNodelet() {}

	void SonarToLaserScanNodelet::onInit()
	{
		boost::mutex::scoped_lock lock(connect_mutex_);
		private_nh_ = getPrivateNodeHandle();

		private_nh_.param<std::string>("target_frame", target_frame_, "");
		private_nh_.param<double>("transform_tolerance", tolerance_, 0.01);
		private_nh_.param<double>("min_height", min_height_, 0.0);
		private_nh_.param<double>("max_height", max_height_, 1.0);
		//之后可能要进行参数的修改。
		private_nh_.param<double>("angle_min", angle_min_, -M_PI / 2.0);
		private_nh_.param<double>("angle_max", angle_max_, M_PI / 2.0);
		private_nh_.param<double>("angle_increment", angle_increment_, M_PI / 360.0);
		private_nh_.param<double>("scan_time", scan_time_, 1.0 / 30.0);
		private_nh_.param<double>("range_min", range_min_, 0.45);
		private_nh_.param<double>("range_max", range_max_, 4.0);

		int concurrency_level;
		private_nh_.param<int>("concurrency_level", concurrency_level, 1);
		private_nh_.param<bool>("use_inf", use_inf_, true);
		//Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
		if (concurrency_level == 1)
		{
			nh_ = getNodeHandle();
		}
		else
		{
			nh_ = getMTNodeHandle();
		}

		// Only queue one pointcloud per running thread
		if (concurrency_level > 0)
		{
			input_queue_size_ = concurrency_level;
		}
		else
		{
			input_queue_size_ = boost::thread::hardware_concurrency();
		}

		// if pointcloud target frame specified, we need to filter by transform availability

		sub_.registerCallback(boost::bind(&SonarToLaserScanNodelet::cloudCb, this, _1));

		pub_ = nh_.advertise<sensor_msgs::LaserScan>("sonarscan", 10,
				boost::bind(&SonarToLaserScanNodelet::connectCb, this),
				boost::bind(&SonarToLaserScanNodelet::disconnectCb, this));
	}
	//这里发现有向scan topic的订阅，所以本程序启动，并订阅从cloud_in的topic的信息进来加以转换，我们在更改时做一个laser to laser的过度。
	void SonarToLaserScanNodelet::connectCb()
	{

		boost::mutex::scoped_lock lock(connect_mutex_);
		if (pub_.getNumSubscribers() > 0 && sub_.getSubscriber().getNumPublishers() == 0)
		{
			NODELET_INFO("Got a subscriber to scan, starting subscriber to sonarscan(base_scan)");
			sub_.subscribe(nh_, "scan", input_queue_size_);//接受原始的laser数据
		}
	}

	void SonarToLaserScanNodelet::disconnectCb()
	{
		boost::mutex::scoped_lock lock(connect_mutex_);
		if (pub_.getNumSubscribers() == 0)
		{
			NODELET_INFO("No subscibers to scan, shutting down subscriber to sonarscan(base_scan)");
			sub_.unsubscribe();
		}
		//fclose(fp);
	}

	void SonarToLaserScanNodelet::cloudCb(const sensor_msgs::LaserScanConstPtr &msg)
	{
		static int steps_1=1,steps_2=0;
		
		int fd;	//读取串口数据，并转化成float数组
		fd = open("/dev/ttyACM1",O_RDWR);  
		if(fd == -1)  
		{  
			perror("serialport error\n"); 
			return; 
		}  
		set_speed(fd,115200);  
		if (set_Parity(fd,8,1,'N') == FALSE)  {  
			printf("Set Parity Error\n");  
			exit (0);  
		} 
		char recv_buf[R];
		int flag_tty=0,flag_range=1;
		int rd_num=0,m=1,k=10,index_max=100;
		float ranges[2]={0.0,0.0};
		float a=-10.0,b=0.0,c=0.0,d=0.0,e=0.0;

		rd_num=read(fd,recv_buf,sizeof(recv_buf));

		if(rd_num>0){
			flag_tty=1;
			recv_buf[rd_num+1]='\0';
			//std::cout<<recv_buf<<std::endl;
			//	fprintf(fp,"rd_num=%d\n",rd_num);
			//char str[]="1.2568 2.0487 3.97\n";
			const char *split=" \n";
			char *p;
			p=strtok(recv_buf,split);

			int i=0;
			while(p!=NULL && i<2)
			{
				ranges[i]=atof(p)*0.75;
				if(ranges[i]<=30 || 80<=ranges[i])
					flag_range=0;
				//printf("ranges[%d]=%.2f\n",i,ranges[i]);
				p=strtok(NULL,split);
				i++;
			}
			//得到数组num


			d=(ranges[1]*ranges[1]+3.5*3.5-ranges[0]*ranges[0])/(7.0*ranges[1]);
			if(fabs(d)<=1)
				a=3.1415926/2-acos(d);
			if(-3.1415926/6<=a && a<=3.1415926/6){
				e=3.1415926/3+acos(d);
				b=sqrt(ranges[1]*ranges[1]+900.0-60*ranges[1]*cos(e));
				c=asin(ranges[1]*sin(e)/b);
			}
			index_max=(msg->angle_max-msg->angle_min)/msg->angle_increment;	 
			k=c/msg->angle_increment;
			b=(b+40)/100.0;
			//printf("%f   %d\n",b,k);
		}
		/*float a1=-10.0,a2;
		  float d1=(ranges[1]*ranges[1]+2*2-ranges[0]*ranges[0])/(4*ranges[1]);
		  float d2=(ranges[1]*ranges[1]+2*2-ranges[2]*ranges[2])/(4*ranges[1]);
		  if(fabs(d1)<=1 && fabs(d2)<=1)
		  {
		  a1=acos(d1)-3.1415926/2;
		  a2=3.1415926/2-acos(d2);
		  }
		  printf("%f,%f\n",a1,a2);
		  printf("%f,%f\n",d1,d2);
		  int k1=(a1-msg->angle_min)/msg->angle_increment;
		  int k2=(a2-msg->angle_min)/msg->angle_increment;*/
		if(-3.1415926/3<=a && a<=3.1415926/3 && flag_range && b<msg->ranges[k])
		{
			if(steps_2==0)
				steps_1++;
			if(steps_1>=5){steps_2=200;}
			if(steps_2>0){steps_1=0;steps_2--;}
			std::cout<<msg->header.stamp<<"           ";	
			printf("sonar scan.b=%f,k=%d\n",b,k);
			//build laserscan output
			sensor_msgs::LaserScan output;

			//设置output的各个参数
			output.header = msg->header;
			if (!target_frame_.empty())
			{
				output.header.frame_id = target_frame_;
			}

			output.angle_min = msg->angle_min;
			output.angle_max = msg->angle_max;
			output.angle_increment = msg->angle_increment;
			output.time_increment = 0.0;
			output.scan_time = msg->scan_time;
			output.range_min = msg->range_min;
			output.range_max = msg->range_max;

			//determine amount of rays to create
			uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

			//determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
			if (use_inf_)
			{
				output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
			}
			else
			{
				output.ranges.assign(ranges_size, output.range_max + 1.0);
			}

			// Iterate through pointcloud
			for(int index=0;index<=index_max;index++){
				//{if(msg->ranges[index]<=b)
					output.ranges[index]=msg->ranges[index];
			}for(int index=k;index<=k+10;index++){
				output.ranges[index]=b-0.1;}
			output.ranges[10]=7.0;
			pub_.publish(output);
		}
		else if(flag_tty && !steps_2){
			if(steps_1>0) steps_1--;
			std::cout<<msg->header.stamp<<"           ";
			printf("laser scan\n");
				//pub_.publish(*msg);
sensor_msgs::LaserScan output;

			//设置output的各个参数
			output.header = msg->header;
			if (!target_frame_.empty())
			{
				output.header.frame_id = target_frame_;
			}

			output.angle_min = msg->angle_min;
			output.angle_max = msg->angle_max;
			output.angle_increment = msg->angle_increment;
			output.time_increment = 0.0;
			output.scan_time = msg->scan_time;
			output.range_min = msg->range_min;
			output.range_max = msg->range_max;

			//determine amount of rays to create
			uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

			//determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
			if (use_inf_)
			{
				output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
			}
			else
			{
				output.ranges.assign(ranges_size, output.range_max + 1.0);
			}

			// Iterate through pointcloud
			for(int index=0;index<=index_max;index++)
				//{if(msg->ranges[index]<=b)
					output.ranges[index]=msg->ranges[index];
			pub_.publish(output);
		}
		if(steps_2>0){steps_2--;}
	}

}

PLUGINLIB_DECLARE_CLASS(sonar_to_laserscan, SonarToLaserScanNodelet, sonar_to_laserscan::SonarToLaserScanNodelet, nodelet::Nodelet);
