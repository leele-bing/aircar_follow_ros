/*
定义一些公用量
*/

#ifndef _COMMON_H_
#define _COMMON_H_

//#define _LOGIC_TEST
//#define _WINDOWS_TEST

#define CAR_SUCCESS 0x00000000
#define CAR_FAILED  0x00000001
/*
故障代码表
0x00010101      CAN报文发送异常(硬件掉线)
0x00010201      CAN报文接收异常(硬件掉线)
0x00010202      没有读到CAN报文
0x00020202      与编码器丢失通信达到上限
0x00020203      阻转报警（立即关闭电机）
0x00030101      线速度非法
0x00030102      角速度非法
0x00040101      控制模式错误
*/


#include<iostream>
#include<chrono>

using namespace std;

typedef unsigned char   byte;
typedef unsigned int    Status;
//typedef chrono::milliseconds    msec_t;

union _TRANSFORM_2_BYTE{
    byte            byte_data[2];
    short           int16_data;
    unsigned short  uint16_data;
};

union _TRANSFORM_4_BYTE{
    byte        byte_data[4];
    int         int32_data;
    unsigned    uint32_data;
    float       float_data;
};

union _TRANSFORM_8_BYTE{
    byte            byte_data[8];
    long int        int64_data;
    unsigned long   uint64_data;
    double          double_data;
};

#endif
