/*
对通信的底层封装
具体实现方法将依据采用的通信模块重写

用于ubuntu系统，使用
*/

#ifndef _COMMUNICATOR_H_
#define _COMMUNICATOR_H_

#include<iostream>
#include"Common.h"

#ifdef _WINDOWS_TEST
#include<windows.h>
#else
#include<unistd.h>
#endif

#include"ControlCAN.h"

using namespace std;

// enum ParamType{MOTOR=0x00, 
//                SETMODE=0x02, 
//                RT_VOLTAGE=0xE1, 
//                RT_CURRENT=0xE2, RT_ROTATE_SPEED, RT_ERROR};


class Communicator{
protected:
    DWORD   device_type;
    DWORD   device_index;
    DWORD   can_index;

protected:
    PVCI_CAN_OBJ    transmit_buffer;            // 发送帧存储区
    PVCI_CAN_OBJ    receive_buffer;             // 接收帧存储区
    const DWORD     RECEIVE_BUFFER_SIZE;        // 缓冲区大小
    
public:
    Communicator();
    ~Communicator();

public:
    Status transmit(unsigned int&, const byte[8]);              // 发送CAN报文
    Status receive(unsigned int&, unsigned int[], byte[][8]);   // 读取接收CAN报文
};



#endif