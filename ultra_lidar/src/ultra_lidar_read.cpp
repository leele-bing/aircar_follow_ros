#include <ros/ros.h>
#include <serial/serial.h>
#include <ultra_lidar/ultra.h>
#include <iostream>
#include <ros/time.h>
using namespace std;
uint8_t Checksum(uint8_t *pData,uint8_t len)
{
    uint8_t sum = 0;
    for (int i = 0; i < len-1; i++)
    {
        sum+=pData[i];
    }

    return sum;
    
}
uint8_t read_data_flag[4] = {0xaa,0xa5,0x01,0xaf};
long long last_get_msg_stamp = 0;
//创建串口对象
serial::Serial ser;
int main(int argc, char **argv)
{
    //创建ros节点
    ros::init(argc, argv, "ultra_serial_node");
    ros::NodeHandle nh;
    //创建发布者
    ros::Publisher ultra_pub = nh.advertise<ultra_lidar::ultra>("ultra_msg", 1000);
    //打开串口设备
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        ROS_ERROR_STREAM(e.what());
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port open");
    }
    else
    {
        return -1;
    }
    ros::Rate loop_rate(6);
    ultra_lidar::ultra ultra_msg;
    while (ros::ok())
    {
   	    
        ser.write(read_data_flag,4);
        //读取串口数据
        size_t n = ser.available();
        if (n != 0)
        {
            // buffer长度可以根据自己的通信协议来修改，可以改大一点如100
            unsigned char buffer[24] = {0};
            n = ser.read(buffer, n);
            //ROS_INFO(&buffer);
            //ROS_INFO("N=%d",n);
            if(n == 24 && buffer[0] == 0xe5 && buffer[1] == 0xe5 && buffer[23] == 0xfe)
            {
                if(buffer[3] = 0x01)
                {
                    float dis_cache = 0;
		            dis_cache = buffer[4]/10.0f;
		            if(dis_cache>1.5)
		            {
			            ultra_msg.distance_array[0] = 0.0;
		            }
		            else
		            {
			            ultra_msg.distance_array[0] = dis_cache;
		            }
                    
                }
                if(buffer[9] = 0x02)
                {
                    float dis_cache = 0;
		            dis_cache = buffer[10]/10.0f;
		            if(dis_cache>1.5)
		            {
			            ultra_msg.distance_array[1] = 0.0;
		            }
		            else
		            {
			            ultra_msg.distance_array[1] = dis_cache;
		            }
                    
                }
                if(buffer[15] = 0x03)
                {
                    float dis_cache = 0;
		            dis_cache = buffer[16]/10.0f;
		            if(dis_cache>1.5)
		            {
			            ultra_msg.distance_array[2] = 0.0;
		            }
		            else
		            {
			            ultra_msg.distance_array[2] = dis_cache;
		            }
                    
                }
                if(buffer[21] = 0x02)
                {
                    float dis_cache = 0;
		            dis_cache = buffer[22]/10.0f;
		            if(dis_cache>1.5)
		            {
			            ultra_msg.distance_array[3] = 0.0;
		            }
		            else
		            {
			            ultra_msg.distance_array[3] = dis_cache;
		            }
                    
                }
                last_get_msg_stamp = ros::Time::now().toNSec();
                ultra_msg.time_out = false;
                ultra_pub.publish(ultra_msg); 
            }
    }
	long long current_stamp = ros::Time::now().toNSec();
	if(current_stamp - last_get_msg_stamp > 500*1000)
	{
	  ultra_msg.time_out = true;
	  ultra_pub.publish(ultra_msg);
	}
        loop_rate.sleep();
    }
}