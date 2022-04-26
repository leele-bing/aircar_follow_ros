#include"MotorDriver.h"
using namespace std;

// code / rad_code_rate / gear_rate = w_motor
MotorDriver::MotorDriver(MotorParameter motor_parm): RECEIVE_BUFFER_SIZE(16){
    gear_rate       = motor_parm.gear_rate;
    rad_code_rate   = motor_parm.rad_code_rate;

    side_sign[0]    = motor_parm.left_sign;
    side_sign[1]    = motor_parm.right_sign;

    slave_id[0]   = motor_parm.left_motor_id;
    slave_id[1]   = motor_parm.right_motor_id;

    obstacle_current_limit = 25.0;

    receive_buffer      = new byte[RECEIVE_BUFFER_SIZE][8];
    receive_id_buffer   = new unsigned int[RECEIVE_BUFFER_SIZE];
    receive_message_number = 0;

    lost_communicate_time = 0;

    present_velocity[0] = 0.0;
    present_velocity[1] = 0.0;
    present_current[0]  = 0.0;
    present_current[1]  = 0.0;
	
Status status_code = CAR_SUCCESS;
    //故障清除
    // cout << "[Motor] Clearing error..." << endl;
    // byte set_error_mode_message[8]  = {0x00, 0x1A, 0x4A, 0x00, 0x00, 0xFF, 0x00, 0x00};
    // _transmitMessage(slave_id[0], set_error_mode_message);
    // _transmitMessage(slave_id[1], set_error_mode_message);
    // cout << "[Motor] Clearing error succeed!" << endl;
    for(unsigned short i=0; i <2; i++){
        transmit_buffer[i][0] = 0x00;
        transmit_buffer[i][1] = 0x1A;
        transmit_buffer[i][2] = 0x4A;
        transmit_buffer[i][3] = 0x00;
        transmit_buffer[i][4] = 0x00;

    }
        transmit_buffer[0][5] = 0x0A;
        transmit_buffer[0][6] = 0x05;
        transmit_buffer[0][7] = 0x05;
        transmit_buffer[1][5] = 0x0A;
        transmit_buffer[1][6] = 0x05;
        transmit_buffer[1][7] = 0x05;
    status_code = _transmitMessage(0);
    cout << status_code << endl;
    status_code = _transmitMessage(1);
    cout << status_code << endl;
    cout << "[Motor] Clearing error succeed!" << endl;
    usleep(1 * 1000);

    //配置加速度
    // cout << "[Motor] Setting speed control mode..." << endl;
    // byte set_speed_mode_message[8]  = {0x00, 0x1A, 0x0A, 0x03, 0x03, 0xFF, 0x00, 0x00};
    // _transmitMessage(slave_id[0], set_speed_mode_message);
    // _transmitMessage(slave_id[1], set_speed_mode_message);
    // cout << "[Motor] Setting speed control mode succeed!" << endl;

//     for(unsigned short i=0; i <2; i++){
//         transmit_buffer[i][2] = 0x0A;
//         transmit_buffer[i][3] = 0x10;
//         transmit_buffer[i][4] = 0x10;
//         transmit_buffer[i][5] = 0xFF;
//         transmit_buffer[i][6] = 0x00;
//         transmit_buffer[i][7] = 0x00;
//     }
//     status_code = _transmitMessage(0);
// cout << status_code << endl;
//     status_code = _transmitMessage(1);
// cout << status_code << endl;
//     cout << "[Motor] Setting speed control mode succeed!" << endl;
// usleep(10 * 1000);
    //配置反馈
    // cout << "[Motor] Setting feedback mode..." << endl;
    // byte set_feedback_message[8]  = {0x00, 0x1A, 0x2E, 0x00, 0x02, 0x0C, 0x00, 0x0A};
    // _transmitMessage(slave_id[0], set_feedback_message);
    // _transmitMessage(slave_id[1], set_feedback_message);
    // cout << "[Motor] Setting feedback mode succeed!" << endl;

    for(unsigned short i=0; i <2; i++){
        transmit_buffer[i][2] = 0x2E;
        transmit_buffer[i][3] = 0x00;
        transmit_buffer[i][4] = 0x02;
        transmit_buffer[i][5] = 0x0C;
        transmit_buffer[i][6] = 0x00;
        transmit_buffer[i][7] = 0x0A;
    }
    status_code = _transmitMessage(0);
cout << status_code << endl;
    status_code = _transmitMessage(1);
cout << status_code << endl;
    cout << "[Motor] Setting feedback mode succeed!" << endl;
usleep(1 * 1000);
    disableMotor();

}

MotorDriver::~MotorDriver(){
    delete[] receive_buffer;
    delete[] receive_id_buffer;
    disableMotor();
}



// void MotorDriver::_rotateSpeedToCode(double motor_velocity[2], byte motor_code[2][2]){
//     /*
//     将double类型的电机转速（rad）转换成16位int型，并拆分成高8位和第8位.
//     */

//     short code_left = 0;
//     short code_right = 0;
//     code_left = (short)(motor_velocity[0] * rad_code_rate * gear_rate * side_sign[0]);
//     code_right = (short)(motor_velocity[1] * rad_code_rate * gear_rate * side_sign[1]);

//     motor_code[0][0] = (code_left & 0xFF00) >> 8;         //左轮高八位
//     motor_code[0][1] = code_left & 0x00FF;              //左轮低八位
//     motor_code[1][0] = (code_right & 0xFF00) >> 8;        //右轮高八位
//     motor_code[1][1] = code_right & 0x00FF;             //右轮低八位
// }

// void MotorDriver::_constructMessage(byte motor_code[2][2], byte motor_message[2][8]){
//     /*
//     构建速控报文
//     */
//     for(unsigned short i=0; i <2; i++){
//         motor_message[i][0] = 0x00;
//         motor_message[i][1] = 0x1A;
//         motor_message[i][2] = 0x06;
//         motor_message[i][3] = motor_code[i][0];
//         motor_message[i][4] = motor_code[i][1];
//         motor_message[i][5] = 0x00;
//         motor_message[i][6] = 0x00;
//         motor_message[i][7] = 0x01;
//     }
// }

void MotorDriver::_constructMessage(double motor_velocity[2]){
    /*
    * 构建速控报文
    */
    for(unsigned short i=0; i <2; i++){
        transmit_buffer[i][2] = 0x06;
        transmit_buffer[i][5] = 0x00;
        transmit_buffer[i][6] = 0x00;
        transmit_buffer[i][7] = 0x01;
    }

    short code_left     = (short)(motor_velocity[0] * rad_code_rate * gear_rate * side_sign[0]);
    short code_right    = (short)(motor_velocity[1] * rad_code_rate * gear_rate * side_sign[1]);

    transmit_buffer[0][3] = (code_left & 0xFF00) >> 8;      // 左轮高八位
    transmit_buffer[0][4] = code_left & 0x00FF;             // 左轮低八位
    transmit_buffer[1][3] = (code_right & 0xFF00) >> 8;     // 右轮高八位
    transmit_buffer[1][4] = code_right & 0x00FF;            // 右轮低八位
}

Status  MotorDriver::velocityControl(double motor_velocity[2]){
    /*
    * 速度控制模式报文编码与发送
    */
    Status status_code = CAR_SUCCESS;
    //byte motor_code[2][2] = {0x00};
    //byte motor_message[2][8] = {0x00};

    //_rotateSpeedToCode(motor_velocity, motor_code);
    //_constructMessage(motor_code, motor_message);
    // 1.速度转换与编码
    _constructMessage(motor_velocity);
    // 2.发送
    status_code = _transmitMessage(0);
    if(status_code != CAR_SUCCESS) return status_code;
    status_code = _transmitMessage(1);
    if(status_code != CAR_SUCCESS) return status_code;

    return status_code;
}

Status MotorDriver::deactive(){
    /*
    * 常用制动停车
    */
    Status status_code = CAR_SUCCESS;
    // 1.编码
    for(unsigned short i=0; i <2; i++){
        transmit_buffer[i][2] = 0x06;
        transmit_buffer[i][3] = 0x00;
        transmit_buffer[i][4] = 0x00;
        transmit_buffer[i][5] = 0x00;
        transmit_buffer[i][6] = 0x00;
        transmit_buffer[i][7] = 0x01;
    }
    // 2.发送
    status_code = _transmitMessage(0);
    if(status_code != CAR_SUCCESS) return status_code;
    status_code = _transmitMessage(1);
    if(status_code != CAR_SUCCESS) return status_code;

    return status_code;
}


Status MotorDriver::disableMotor(){
    /*
    * 关闭电机，使车随动
    */
    Status status_code = CAR_SUCCESS;
    // 1.编码
    for(unsigned short i=0; i <2; i++){
        transmit_buffer[i][2] = 0x06;
        transmit_buffer[i][3] = 0x00;
        transmit_buffer[i][4] = 0x00;
        transmit_buffer[i][5] = 0x00;
        transmit_buffer[i][6] = 0x00;
        transmit_buffer[i][7] = 0x00;
    }
    // 2.发送
    _transmitMessage(0);
    if(status_code != CAR_SUCCESS) return status_code;
    _transmitMessage(1);
    if(status_code != CAR_SUCCESS) return status_code;
    
    return status_code;
}

Status MotorDriver::enableMotor(){
    /*
    * 启动电机
    */
    Status status_code = CAR_SUCCESS;
    // 1.编码
    for(unsigned short i=0; i <2; i++){
        transmit_buffer[i][2] = 0x06;
        transmit_buffer[i][3] = 0x00;
        transmit_buffer[i][4] = 0x00;
        transmit_buffer[i][5] = 0x00;
        transmit_buffer[i][6] = 0x00;
        transmit_buffer[i][7] = 0x01;
    }
    // 2.发送
    _transmitMessage(0);
    if(status_code != CAR_SUCCESS) return status_code;
    _transmitMessage(1);
    if(status_code != CAR_SUCCESS) return status_code;

    return status_code;
}

Status MotorDriver::_transmitMessage(unsigned int transmit_index){
    /*
    * 发送CAN报文的底层实现
    */
    return communicator.transmit(slave_id[transmit_index], transmit_buffer[transmit_index]);
}


Status MotorDriver::loopMonitor(){
    /*
    * 循环监控运行状态
    */
    Status status_code = CAR_SUCCESS;
    // 1.读取消息
    status_code = _receiveMessage();
    if(status_code != CAR_SUCCESS){
        //未成功读取报文
        lost_communicate_time += 1;
        if(lost_communicate_time >= 10){
            //与编码器丢失通信
            return 0x00020202;
        }
        //少数两次丢报文，不报错
        return CAR_SUCCESS;
    }
    // 2.成功读取，分析报文
    lost_communicate_time = 0;
    status_code = _analysisMessage();
    if(status_code != CAR_SUCCESS){
        //读到故障报文
        return status_code;
    }

    // 3.安全检查
   // status_code = _obstacleCheck();
    // 发生阻转，关闭电机后上报故障
    return status_code;
}

Status MotorDriver::_receiveMessage(){
    /*
    * 接收报文的底层实现
    */
    return communicator.receive(receive_message_number, receive_id_buffer, receive_buffer);
}

Status MotorDriver::_analysisMessage(){
    /*
    对已读取的报文组进行逐条分析
    */
    for(unsigned int i=0; i<receive_message_number; i++){
        // 1.分析ID，写入对应电机
        unsigned short source_index = 0;
        if(receive_id_buffer[i] == slave_id[0]) source_index = 0;
        else if(receive_id_buffer[i] == slave_id[1]) source_index = 1;
        else continue;
        // 2.报文类别分析
        switch(receive_buffer[i][1]){
            case 0x1B:{
                //写反馈正常 暂不处理
                //return CAR_SUCCESS;
                cout << "ID: " << hex << (int)receive_id_buffer[i] << "\tmessage: ";
                for(unsigned k=0; k<8; k++){
                    cout << hex << (int)receive_buffer[i][k] << " "	;	
                }
                cout << endl;
                break;
            }
            case 0x1C:{
                //写反馈故障 暂不处理
                //return CAR_FAILED;
                cout << "ID: " << hex << (int)receive_id_buffer[i] << "\tmessage: ";
                for(unsigned k=0; k<8; k++){
                    cout << hex << (int)receive_buffer[i][k] << " "	;	
                }
                cout << endl;
                break;
            }
            case 0x2B:{
                //读反馈正常 暂不处理
                cout << "ID: " << hex << (int)receive_id_buffer[i] << "\tmessage: ";
                for(unsigned k=0; k<8; k++){
                    cout << hex << (int)receive_buffer[i][k] << " "	;	
                }
                cout << endl;
                break;
            }
            case 0x2C:{
                //读反馈故障 暂不处理
                cout << "ID: " << hex << (int)receive_id_buffer[i] << "\tmessage: ";
                for(unsigned k=0; k<8; k++){
                    cout << hex << (int)receive_buffer[i][k] << " "	;	
                }
                cout << endl;
                break;
            }
            case 0x88:{
                //自动反馈信息
                _onDealFeedback(source_index, receive_buffer[i]);
                break;
            }
            case 0xFF:{
                //自动故障信息
                cout << "ID: " << hex << (int)receive_id_buffer[i] << "\tmessage: ";
                for(unsigned k=0; k<8; k++){
                    cout << hex << (int)receive_buffer[i][k] << " "	;	
                }
                cout << endl;
                disableMotor();
                return (unsigned int)((receive_id_buffer[i] << 8) + receive_buffer[i][4]);
            }
            default:{
                cout << "ID: " << hex << (int)receive_id_buffer[i] << "\tmessage: ";
                for(unsigned k=0; k<8; k++){
                    cout << hex << (int)receive_buffer[i][k] << " "	;	
                }
                cout << endl;
                break;
            }
        }
    }

    return CAR_SUCCESS;
}


void MotorDriver::_onDealFeedback(unsigned short& source_index, byte message[8]){
    /*
    * 处理自动反馈报文
    */
    for(unsigned short i=2; i<=5; i+=3){
        switch(message[i]){
            case 0xE2:{
                // 电流反馈
                present_current[source_index] = (short)((message[i + 1] << 8) + message[i + 2]) / 100.0;
                break;
            }
            case 0xE4:{
                // 转速反馈
                //_codeToRotateSpeed(source_index, &message[i + 1]);
                short motor_code = (short)((message[i + 1] << 8) + message[i + 2]);
                present_velocity[source_index] = motor_code / rad_code_rate / gear_rate * side_sign[source_index];
                break;
            }
            default:{
                break;
            }
        }
    }
}

// void _onDealReadSuccess(){}

// void _onDealReadFailed(){}

// void _onDealWriteSuccess(){}

// void _onDealWriteFailed(){}



void MotorDriver::_codeToRotateSpeed(unsigned short& source_index, byte motor_code[2]){
    /*
    * 将byte型马达转速编码转成实际转速（rad/s）
    */
    present_velocity[source_index] = (short)((motor_code[0] << 8) + motor_code[1]) / rad_code_rate / gear_rate * side_sign[source_index];
}

Status MotorDriver::_obstacleCheck(){
    /*
    * 阻转安全检查，一旦发现阻转，立刻关闭电机
    */
    // if(present_current[0] > obstacle_current_limit &&
    //    present_velocity[0] < obstacle_velocity_limit){
    //         disableMotor();
    //         return 0x00020203;}
    // if(present_current[1] > obstacle_current_limit &&
    //    present_velocity[1] < obstacle_velocity_limit){
    //         disableMotor();
    //         return 0x00020203;}
    if(present_current[0] > obstacle_current_limit){
            disableMotor();
            return 0x00020203;
    }
    if(present_current[1] > obstacle_current_limit){
            disableMotor();
            return 0x00020203;
    }
    return CAR_SUCCESS;
}


double* MotorDriver::getPresentCurrent(){
    return present_current;
}

double* MotorDriver::getPresentVelocity(){
    return present_velocity;
}


Status MotorDriver::clearError(){
    /*
    * 故障清除
    */
    Status status_code = CAR_SUCCESS;

    for(unsigned short i=0; i <2; i++){
        transmit_buffer[i][0] = 0x00;
        transmit_buffer[i][1] = 0x1A;
        transmit_buffer[i][2] = 0x4A;
        transmit_buffer[i][3] = 0x00;
        transmit_buffer[i][4] = 0x00;
        transmit_buffer[i][5] = 0xFF;
        transmit_buffer[i][6] = 0x00;
        transmit_buffer[i][7] = 0x00;
    }
    status_code = _transmitMessage(0);
    if(status_code != CAR_SUCCESS){
        return status_code;
    }
    status_code = _transmitMessage(1);
    if(status_code != CAR_SUCCESS){
        return status_code;
    }
    return CAR_SUCCESS;
}
