#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <ros/time.h>
#include <uwb_nooploop_aoa_serial/UwbAOA0.h>
#include <uwb_nooploop_aoa_serial/UwbAOA1.h>
#include <uwb_nooploop_aoa_serial/UwbAOA2.h>
#include <uwb_nooploop_aoa_serial/UwbAOA3.h>
#include <stdio.h>

using namespace std;
//创建串口对象
serial::Serial ser;
long long last_get_aoa0_msg_stamp = 0;
long long last_get_aoa1_msg_stamp = 0;
long long last_get_aoa2_msg_stamp = 0;

uint8_t verifyCheckSum(uint8_t *data, int32_t length)
{
    uint8_t sum = 0;
    for (int32_t i = 0; i < length - 1; ++i)
    {
        sum += data[i];
    }
    return sum == data[length - 1];
}

int main(int argc, char *argv[])
{
    //创建ros节点
    ros::init(argc, argv, "uwb_aoa_serial_node");
    ros::NodeHandle nh;
    //创建发布者
    ros::Publisher uwb0_pub = nh.advertise<uwb_nooploop_aoa_serial::UwbAOA0>("uwb_aoa0_msg", 1000);
    ros::Publisher uwb1_pub = nh.advertise<uwb_nooploop_aoa_serial::UwbAOA1>("uwb_aoa1_msg", 1000);
    ros::Publisher uwb2_pub = nh.advertise<uwb_nooploop_aoa_serial::UwbAOA2>("uwb_aoa2_msg", 1000);
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(921600);
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
    uwb_nooploop_aoa_serial::UwbAOA0 uwb_aoa0_msg;
    uwb_nooploop_aoa_serial::UwbAOA1 uwb_aoa1_msg;
    uwb_nooploop_aoa_serial::UwbAOA2 uwb_aoa2_msg;
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
	size_t n = 0;
	try	
	{
	            //读取串口数据
            n = ser.available();
	}
	catch (serial::IOException &e)
       { 
            ROS_ERROR_STREAM(e.what());
			uwb_aoa0_msg.time_out = true;
			uwb_aoa1_msg.time_out = true;
			uwb_aoa2_msg.time_out = true;
			uwb0_pub.publish(uwb_aoa0_msg);
			uwb1_pub.publish(uwb_aoa1_msg);
			uwb2_pub.publish(uwb_aoa2_msg);
        	return -1;
        }

        if (n != 0)
        {
            // buffer长度可以根据自己的通信协议来修改，可以改大一点如100
            unsigned char buffer[66] = {0};
            n = ser.read(buffer, n);
            if (n == 66)
            {
                ROS_INFO_STREAM("receive 66 bytes");
            }
            else if (n == 55)
            {
                if (buffer[0] == 0x55 && buffer[1] == 0x07 && (uint16_t((buffer[3] & 0xff) << 8 | buffer[2] & 0xff)) == 0x37)
                {
                    if (verifyCheckSum(buffer, 55) == true)
                    {
                        if (buffer[22] == 0x00)
                        {
                            uwb_aoa0_msg.role = buffer[21];
                            uwb_aoa0_msg.id = buffer[22];
                            int32_t dis = 0;
                            dis = (buffer[25] << 24 | buffer[24] << 16 | buffer[23] << 8) / 256;
                            uwb_aoa0_msg.distance = dis / 1000.0f;
                            int16_t angl = 0;
                            angl = (buffer[27] << 8 | buffer[26]);
                            uwb_aoa0_msg.angle = angl / 100.0f;
                            uwb_aoa0_msg.fp_rssi = (buffer[28] / -2.0);
                            uwb_aoa0_msg.rx_rssi = (buffer[29] / -2.0);
			                uwb_aoa0_msg.time_out = false;
                            last_get_aoa0_msg_stamp = ros::Time::now().toNSec();
                            uwb0_pub.publish(uwb_aoa0_msg);
                        }
                        if (buffer[33] == 0x01)
                        {
                            uwb_aoa1_msg.role = buffer[32];
                            uwb_aoa1_msg.id = buffer[33];
                            int32_t dis = 0;
                            dis = (buffer[36] << 24 | buffer[35] << 16 | buffer[34] << 8) / 256;
                            uwb_aoa1_msg.distance = dis / 1000.0f;
                            int16_t angl = 0;
                            angl = (buffer[38] << 8 | buffer[37]);
                            uwb_aoa1_msg.angle = angl / 100.0f;
                            uwb_aoa1_msg.fp_rssi = (buffer[39] / -2.0);
                            uwb_aoa1_msg.rx_rssi = (buffer[40] / -2.0);
		  	    uwb_aoa1_msg.time_out = false;
                            last_get_aoa1_msg_stamp = ros::Time::now().toNSec();
                            uwb1_pub.publish(uwb_aoa1_msg);
                        }
                        else if (buffer[33] == 0x02)
                        {
                            uwb_aoa2_msg.role = buffer[32];
                            uwb_aoa2_msg.id = buffer[33];
                            int32_t dis = 0;
                            dis = (buffer[36] << 24 | buffer[35] << 16 | buffer[34] << 8) / 256;
                            uwb_aoa2_msg.distance = dis / 1000.0f;
                            int16_t angl = 0;
                            angl = (buffer[38] << 8 | buffer[37]);
                            uwb_aoa2_msg.angle = angl / 100.0f;
                            uwb_aoa2_msg.fp_rssi = (buffer[39] / -2.0);
                            uwb_aoa2_msg.rx_rssi = (buffer[40] / -2.0);
                            uwb_aoa2_msg.time_out = false;
                            last_get_aoa2_msg_stamp = ros::Time::now().toNSec();
                            uwb2_pub.publish(uwb_aoa2_msg);
                        }
                        if (buffer[44] == 0x01)
                        {
                            uwb_aoa1_msg.role = buffer[43];
                            uwb_aoa1_msg.id = buffer[44];
                            int32_t dis = 0;
                            dis = (buffer[47] << 24 | buffer[46] << 16 | buffer[45] << 8) / 256;
                            uwb_aoa1_msg.distance = dis / 1000.0f;
                            int16_t angl = 0;
                            angl = (buffer[49] << 8 | buffer[48]);
                            uwb_aoa1_msg.angle = angl / 100.0f;
                            uwb_aoa1_msg.fp_rssi = (buffer[50] / -2.0);
                            uwb_aoa1_msg.rx_rssi = (buffer[51] / -2.0);
			    uwb_aoa1_msg.time_out = false;
                            last_get_aoa1_msg_stamp = ros::Time::now().toNSec();
                            uwb1_pub.publish(uwb_aoa1_msg);
                        }
                        else if (buffer[44] == 0x02)
                        {
                            uwb_aoa2_msg.role = buffer[43];
                            uwb_aoa2_msg.id = buffer[44];
                            int32_t dis = 0;
                            dis = (buffer[47] << 24 | buffer[46] << 16 | buffer[45] << 8) / 256;
                            uwb_aoa2_msg.distance = dis / 1000.0f;
                            int16_t angl = 0;
                            angl = (buffer[49] << 8 | buffer[48]);
                            uwb_aoa2_msg.angle = angl / 100.0f;
                            uwb_aoa2_msg.fp_rssi = (buffer[50] / -2.0);
                            uwb_aoa2_msg.rx_rssi = (buffer[51] / -2.0);
			    uwb_aoa2_msg.time_out = false;
                            last_get_aoa2_msg_stamp = ros::Time::now().toNSec();
                            uwb2_pub.publish(uwb_aoa2_msg);
                        }
                    }
                }
            }
            else if (n == 44)
            {
                if (buffer[0] == 0x55 && buffer[1] == 0x07 && (uint16_t((buffer[3] & 0xff) << 8 | buffer[2] & 0xff)) == 0x2C)
                {
                    if (verifyCheckSum(buffer, 44) == true)
                    {
                        if (buffer[22] == 0x00)
                        {
                            uwb_aoa0_msg.role = buffer[21];
                            uwb_aoa0_msg.id = buffer[22];
                            int32_t dis = 0;
                            dis = (buffer[25] << 24 | buffer[24] << 16 | buffer[23] << 8) / 256;
                            uwb_aoa0_msg.distance = dis / 1000.0f;
                            int16_t angl = 0;
                            angl = (buffer[27] << 8 | buffer[26]);
                            uwb_aoa0_msg.angle = angl / 100.0f;
                            uwb_aoa0_msg.fp_rssi = (buffer[28] / -2.0);
                            uwb_aoa0_msg.rx_rssi = (buffer[29] / -2.0);
                            uwb_aoa0_msg.time_out = false;
                            last_get_aoa0_msg_stamp = ros::Time::now().toNSec();
                            uwb0_pub.publish(uwb_aoa0_msg);
                        }
                        if (buffer[33] == 0x01)
                        {
                            uwb_aoa1_msg.role = buffer[32];
                            uwb_aoa1_msg.id = buffer[33];
                            int32_t dis = 0;
                            dis = (buffer[36] << 24 | buffer[35] << 16 | buffer[34] << 8) / 256;
                            uwb_aoa1_msg.distance = dis / 1000.0f;
                            int16_t angl = 0;
                            angl = (buffer[38] << 8 | buffer[37]);
                            uwb_aoa1_msg.angle = angl / 100.0f;
                            uwb_aoa1_msg.fp_rssi = (buffer[39] / -2.0);
                            uwb_aoa1_msg.rx_rssi = (buffer[40] / -2.0);
                            uwb_aoa1_msg.time_out = false;
                            last_get_aoa1_msg_stamp = ros::Time::now().toNSec();
                            uwb1_pub.publish(uwb_aoa1_msg);
                        }
                        else if (buffer[33] == 0x02)
                        {
                            uwb_aoa2_msg.role = buffer[32];
                            uwb_aoa2_msg.id = buffer[33];
                            int32_t dis = 0;
                            dis = (buffer[36] << 24 | buffer[35] << 16 | buffer[34] << 8) / 256;
                            uwb_aoa2_msg.distance = dis / 1000.0f;
                            int16_t angl = 0;
                            angl = (buffer[38] << 8 | buffer[37]);
                            uwb_aoa2_msg.angle = angl / 100.0f;
                            uwb_aoa2_msg.fp_rssi = (buffer[39] / -2.0);
                            uwb_aoa2_msg.rx_rssi = (buffer[40] / -2.0);
                            uwb_aoa2_msg.time_out = false;
                            last_get_aoa2_msg_stamp = ros::Time::now().toNSec();
                            uwb2_pub.publish(uwb_aoa2_msg);
                        }
                    }
                }
            }
            else if (n == 33)
            {
                if (buffer[0] == 0x55 && buffer[1] == 0x07 && (uint16_t((buffer[3] & 0xff) << 8 | buffer[2] & 0xff)) == 0x21)
                {
                    if (verifyCheckSum(buffer, 33) == true)
                    {
                        if (buffer[22] == 0x00)
                        {
                            uwb_aoa0_msg.role = buffer[21];
                            uwb_aoa0_msg.id = buffer[22];
                            int32_t dis = 0;
                            dis = (buffer[25] << 24 | buffer[24] << 16 | buffer[23] << 8) / 256;
                            uwb_aoa0_msg.distance = dis / 1000.0f;
                            int16_t angl = 0;
                            angl = (buffer[27] << 8 | buffer[26]);
                            uwb_aoa0_msg.angle = angl / 100.0f;
                            uwb_aoa0_msg.fp_rssi = (buffer[28] / -2.0);
                            uwb_aoa0_msg.rx_rssi = (buffer[29] / -2.0);
                            uwb_aoa0_msg.time_out = false;
                            last_get_aoa0_msg_stamp = ros::Time::now().toNSec();
                            uwb0_pub.publish(uwb_aoa0_msg);
                        }
                    }
                }
            }
        }
        long long current_stamp = ros::Time::now().toNSec();
        // ROS_INFO_STREAM(current_stamp);
        // ROS_ERROR_STREAM(last_get_aoa0_msg_stamp);
        if(current_stamp - last_get_aoa0_msg_stamp > 200 * 1000 *1000)
        {
            uwb_aoa0_msg.time_out = true;
            ROS_ERROR_STREAM("time out");
            ROS_INFO("time:%d",current_stamp - last_get_aoa0_msg_stamp);
            uwb0_pub.publish(uwb_aoa0_msg);
        }
        if(current_stamp - last_get_aoa1_msg_stamp > 200 * 1000 *1000)
        {
            uwb_aoa1_msg.time_out = true;
            uwb1_pub.publish(uwb_aoa1_msg);
        }
        if(current_stamp - last_get_aoa2_msg_stamp > 200 * 1000 *1000)
        {
            uwb_aoa2_msg.time_out = true;
            uwb2_pub.publish(uwb_aoa2_msg);
        }
        loop_rate.sleep();
    }
}
