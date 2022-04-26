#include <ros/ros.h>
#include <serial/serial.h>
#include <ultra_serial/ultra.h>
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

long long last_get_msg_stamp = 0;
//创建串口对象
serial::Serial ser;
int main(int argc, char **argv)
{
    //创建ros节点
    ros::init(argc, argv, "ultra_serial_node");
    ros::NodeHandle nh;
    //创建发布者
    ros::Publisher ultra_pub = nh.advertise<ultra_serial::ultra>("ultra_msg", 1000);
    //打开串口设备
    try
    {
        ser.setPort("/dev/ttyUSB1");
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
    ros::Rate loop_rate(10);
    ultra_serial::ultra ultra_msg;
    while (ros::ok())
    {
	
   	uint8_t read_data_flag = 1;
	size_t n = 0;
   	try
   	{
            ser.write(&read_data_flag,1);
            //读取串口数据
            n = ser.available();
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM(e.what());
            ultra_msg.time_out = true;
            ultra_pub.publish(ultra_msg); 
            return -1;
        }
        if (n != 0)
        {
            // buffer长度可以根据自己的通信协议来修改，可以改大一点如100
            unsigned char buffer[10] = {0};
            n = ser.read(buffer, n);
            if(buffer[0] == 0xff)
            {
		if(buffer[9] = Checksum(buffer,9))
		{
		    float dis_cache = 0;
		    dis_cache = ((buffer[1] & 0xff)<< 8 | buffer[2] & 0xff)/1000.0;
		    if(dis_cache>1.5)
		    {
			ultra_msg.distance_array[0] = 0.0;
		    }
		    else
		    {
			ultra_msg.distance_array[0] = dis_cache;
		    }
		    dis_cache = 0;
		    dis_cache = ((buffer[3] & 0xff)<< 8 | buffer[4] & 0xff)/1000.0;
		     if(dis_cache>1.5)
		    {
			ultra_msg.distance_array[1] = 0.0;
		    }
		    else
		    {
			ultra_msg.distance_array[1] = dis_cache;
		    }
		    dis_cache = 0;
		    dis_cache = ((buffer[5] & 0xff)<< 8 | buffer[6] & 0xff)/1000.0;
		     if(dis_cache>1.5)
		    {
			ultra_msg.distance_array[2] = 0.0;
		    }
		    else
		    {
			ultra_msg.distance_array[2] = dis_cache;
		    }
		    dis_cache = 0;
		    dis_cache = ((buffer[7] & 0xff)<< 8 | buffer[8] & 0xff)/1000.0;
		     if(dis_cache>1.5)
		    {
			ultra_msg.distance_array[3] = 0.0;
		    }
		    else
		    {
			ultra_msg.distance_array[3] = dis_cache;
		    }
	            ultra_msg.time_out = false;
		    last_get_msg_stamp = ros::Time::now().toNSec();
                    ultra_pub.publish(ultra_msg); 
		}

            }
        }
	long long current_stamp = ros::Time::now().toNSec();
	if(current_stamp - last_get_msg_stamp > 300*1000*1000)
	{
	  ultra_msg.time_out = true;
	  ultra_pub.publish(ultra_msg);
	}
        loop_rate.sleep();
    }
}
