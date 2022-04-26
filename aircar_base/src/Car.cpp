#include"Car.h"

using namespace std;

Car::Car(CarModelParameter carModelParameter, MotorParameter motorParameter) :
                                                        car_model(carModelParameter),
                                                        motor_driver(motorParameter){
    car_state = _IDLE;
}

Car::~Car(){
    car_state = _IDLE;
    toIdleMode();
}


Status Car::velControl(double target_velocity[2]){
    /*
      速度控制
    */
    //检查模式
    //cout << "[CTRL ] Velocity control." << endl;
    if(car_state != _ONCONTROL){ 
        //cout << "[CTRL ] Error control mode." << endl;
        return 0x00040101;
    }

    Status  status_code = CAR_SUCCESS;

    //1.检查速度值合法性
    status_code = car_model.checkLimit(target_velocity);
    if(status_code != CAR_SUCCESS) return status_code;

    //2.速度值转算
    double motor_velocity[2] = {0.0, 0.0};
    car_model.carVtoMotorW(target_velocity, motor_velocity);

    //3.下达指令
    status_code = motor_driver.velocityControl(motor_velocity);
    if(status_code != CAR_SUCCESS) return status_code;

    //4.更新里程计状态
    car_model.updateVelocity(target_velocity);

    //cout << "[CTRL ] Velocity control succeed." << endl;
    return CAR_SUCCESS;
}



Status Car::deactive(){
    /*
    常用制动 停车
    */
    //cout << "[CTRL ] Deactive the car." << endl;
    //检查模式
    if(car_state != _ONCONTROL){ 
        //cout << "[CTRL ] Error control mode." << endl;
        return 0x00040101;
    }
    Status  status_code = CAR_SUCCESS;

    //1.下达指令
    status_code = motor_driver.deactive();
    if(status_code) return status_code;

    //2.更新里程计状态
    double target_velocity[2] = {0.0, 0.0};
    car_model.updateVelocity(target_velocity);

    //cout << "[CTRL ] Deactive car succeed." << endl;

    return CAR_SUCCESS;
} 



Status Car::getOrderOdom(double odom[5]){
    /*
    * 获取指令里程计信息
    * idle状态里程计无效！
    */
    if(car_state == _ONCONTROL){
        //速控状态正常读取
        car_model.getOrderOdom(odom);
        return CAR_SUCCESS;
    } else{
        //其他状态返回全零
        odom[0] = 0.0;
        odom[1] = 0.0;
        odom[2] = 0.0;
        odom[3] = 0.0;
        odom[4] = 0.0;
        return CAR_FAILED;
    } 
}




void Car::initOdom(){
    /*
    里程计初始化
    */
    car_model.initOdom();
}


void Car::resetOdom(double odom_pose[3]){
    /*
    * 依据给定位姿重设里程计
    */
    if(car_state != _ONCONTROL){
        // 仅在速控模式下有效
        //cout << "[CTRL ] Error control mode." << endl;
        return;
    }

    car_model.resetOdom(odom_pose);
}



Status Car::toIdleMode(){
    /*
      转到空闲随动模式
    */
    //cout << "[CTRL ] Change to idle mode." << endl;
    Status  status_code = CAR_SUCCESS;

    //1.下达指令
    status_code = motor_driver.disableMotor();
    if(status_code) {
        //cout << "[CTRL ] Change to idle mode failed." << endl;
        return status_code;
    }

    //2.更新里程计状态
    double target_velocity[2] = {0.0, 0.0};
    car_model.updateVelocity(target_velocity);
    car_model.initOdom();
    car_state = _IDLE;

    //cout << "[CTRL ] Change to idle mode succeed." << endl;
    return CAR_SUCCESS;
}



Status Car::toControlMode(){
    /*
      转到命令控制模式
    */
    //cout << "[CTRL ] Change to control mode." << endl;
    Status  status_code = CAR_SUCCESS;

    //1.下达指令
    status_code = motor_driver.enableMotor();
    if(status_code) {
        //cout << "[CTRL ] Change to control mode failed." << endl;
        return status_code;
    }

    //2.更新里程计状态
    double target_velocity[2] = {0.0, 0.0};
    car_model.updateVelocity(target_velocity);
    car_model.initOdom();
    car_state = _ONCONTROL;

    //cout << "[CTRL ] Change to control mode succeed." << endl;
    return CAR_SUCCESS;
}


Status Car::loopUpdate(){
    /*
    循环更新状态参数
    */
    //1.刷新电机参数
    Status  status_code = CAR_SUCCESS;
    status_code = motor_driver.loopMonitor();
    if(status_code != CAR_SUCCESS){
        //故障状态，此时已经停机，清零odom和速度
        car_state = _ERROR;

        double target_velocity[2] = {0.0, 0.0};
        car_model.updateVelocity(target_velocity);
        car_model.initOdom();
        return status_code;
    }

    //2.使用电机参数解算车速
    double car_velocity[2] = {0.0, 0.0};
    double* motor_velocity;
    motor_velocity = motor_driver.getPresentVelocity();
    car_model.motorWtoCarV(motor_velocity, car_velocity);

    //更新里程计
    car_model.updateWheelOdom(car_velocity);
    return CAR_SUCCESS;
}

Status Car::getWheelOdom(double odom[5]){
    /*
    * 获取轮式里程计信息
    * idle状态里程计无效！
    */
    if(car_state == _ONCONTROL){
        //速控状态读取有效
        car_model.getWheelOdom(odom);
        return CAR_SUCCESS;
    } else{
        //其他状态返回全零
        odom[0] = 0.0;
        odom[1] = 0.0;
        odom[2] = 0.0;
        odom[3] = 0.0;
        odom[4] = 0.0;
        return CAR_FAILED;
    } 
}

double* Car::getCurrent(){
    /*
    获取马达电流数据
    */
    return motor_driver.getPresentCurrent();
}

double* Car::getVelocity(){
    /*
    获取马达电流数据
    */
    return motor_driver.getPresentVelocity();
}

CarState Car::getState(){
    return car_state;
}