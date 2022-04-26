#include"Communicator.h"

using namespace std;

Communicator::Communicator(): RECEIVE_BUFFER_SIZE(64){
    //CAN参数设置
    this->device_type       = VCI_USBCAN2;
    this->device_index      = 0;
    this->can_index         = 0;
    
    transmit_buffer     = new VCI_CAN_OBJ;
    receive_buffer      = new VCI_CAN_OBJ[RECEIVE_BUFFER_SIZE];

    DWORD can_feedback;

    // 开启设备
    while(1) {
        can_feedback = VCI_OpenDevice(device_type, device_index, 0);
        if(can_feedback == 1) {
            cout << "[CAN  ] Open CAN device success!" << endl;
            break;
        }else if(can_feedback == 0) {
            cout << "[CAN  ] Open CAN device failed!" << endl;
            cout << "[CAN  ] Retry in 5s..." << endl;
        }else{
            cout << "[CAN  ] No USB device found!" << endl;
            cout << "[CAN  ] Retry in 5s..." << endl;
        }
        sleep(5);
    }
    

    // CAN初始化
    VCI_INIT_CONFIG can_init_config = {         // CAN初始化参数
        AccCode  : 0,                           //
        AccMask  : 0xFFFFFFFF,                  //
        Reserved : 0,
        Filter   : 1,                           //
        Timing0  : 0x00,                        //
        Timing1  : 0x1C,                        //
        Mode     : 0                            //
    };

    while(1) {
        can_feedback = VCI_InitCAN(device_type, device_index, can_index, &can_init_config);
        if(can_feedback == 1) {
            cout << "[CAN  ] Init CAN success!" << endl;
            break;
        }else if(can_feedback == 0) {
            cout << "[CAN  ] Init CAN failed!" << endl;
            cout << "[CAN  ] Retry in 5s..." << endl;
        }else{
            cout << "[CAN  ] No USB device found!" << endl;
            cout << "[CAN  ] Retry in 5s..." << endl;
        }
        sleep(5);
    }

    // 开启CAN
    while(1) {
        can_feedback = VCI_StartCAN(device_type, device_index, can_index);
        if(can_feedback == 1) {
            cout << "[CAN  ] Start CAN success!" << endl;
            break;
        }else if(can_feedback == 0) {
            cout << "[CAN  ] Start CAN failed!" << endl;
            cout << "[CAN  ] Retry in 5s..." << endl;
        }else{
            cout << "[CAN  ] No USB device found!" << endl;
            cout << "[CAN  ] Retry in 5s..." << endl;
        }
        sleep(5);
    }

}


Communicator::~Communicator(){
    delete transmit_buffer;
    delete[] receive_buffer;
    
    DWORD can_feedback;

    // 关闭CAN
    while(1) {
        can_feedback = VCI_CloseDevice(device_type, device_index);
        if(can_feedback == 1) {
            cout << "[CAN  ] Close CAN device success!" << endl;
            break;
        }else if(can_feedback == 0) {
            cout << "[CAN  ] Close CAN device failed!" << endl;
            cout << "[CAN  ] Retry in 5s..." << endl;
        }else{
            cout << "[CAN  ] No USB device found!" << endl;
            cout << "[CAN  ] Retry in 5s..." << endl;
        }
        sleep(5);
    }
}


Status Communicator::transmit(unsigned int& message_id, const byte message[8]){
    /*
    发送CAN报文，固定为8位
    */
    DWORD can_feedback;

    // 构建数据帧
    // VCI_CAN_OBJ can_frame;
    // can_frame.ID           = message_id;
    // can_frame.SendType     = 0;
    // can_frame.RemoteFlag   = 0;
    // can_frame.ExternFlag   = 0;
    // can_frame.DataLen      = 8;
    // for(byte i = 0; i < 8; i ++){
    //     can_frame.Data[i] = message[i];
    // }

    transmit_buffer->ID         = message_id;
    transmit_buffer->SendType   = 0;
    transmit_buffer->RemoteFlag = 0;
    transmit_buffer->ExternFlag = 0;
    transmit_buffer->DataLen    = 8;
    for(byte i = 0; i < 8; i ++){
        transmit_buffer->Data[i] = message[i];
    }

    // 发送数据帧
    can_feedback = VCI_Transmit(device_type, device_index, can_index, transmit_buffer, 1);

    if(can_feedback == -1){
        cout << "[CAN  ] No USB device found!" << endl;
        return 0x00010101;
    } else{
        return CAR_SUCCESS;
    }
}


Status Communicator::receive(unsigned int& message_number, unsigned int message_id_list[], byte message_list[][8]){
    /*
    * 接收消息，从缓冲区读取消息
    */
    DWORD can_feedback;

    //获取缓冲区报文数量
    can_feedback = VCI_GetReceiveNum(device_type, device_index, can_index);
    if(can_feedback > 0){
        message_number = can_feedback;
        message_number = 0;
    } else{
        return 0x00010202;
    }

    //读取报文
    can_feedback = VCI_Receive(device_type, device_index, can_index, receive_buffer, RECEIVE_BUFFER_SIZE, 0);
    if(can_feedback > 0){
        message_number = can_feedback;
        for(short i=0; i<message_number; i++){
            message_id_list[i] = receive_buffer[i].ID;
            // cout << "ID=" << (short)message_id_list[i] << "  "; 
            for(short j=0; j<8; j++){
                message_list[i][j] = receive_buffer[i].Data[j];
                // cout << hex << (short)message_list[i][j] << " ";
            }
            // cout << endl;
        }
        return CAR_SUCCESS;
    } else if(can_feedback == 0){
        message_number = 0;
        return 0x00010202;
    } else{
        return  0x00010201;
    }

}
