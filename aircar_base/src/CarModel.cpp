#include"CarModel.h"

using namespace std;



CarModel::CarModel(CarModelParameter carModelParameter){
    // cout << "[Model] Setting parameters..." << endl;

    r_wheel     = carModelParameter.wheel_radiu;
    l_tread     = carModelParameter.tread_length;
    v_ulimit    = carModelParameter.linear_velocity_up_limit;
    w_ulimit    = carModelParameter.rotate_velocity_up_limit;
    v_llimit    = carModelParameter.linear_velocity_low_limit;
    w_llimit    = carModelParameter.rotate_velocity_low_limit;

    cout << "[Motor] Setting parameters succeed!" << endl;
    
    v_current   = 0.0;
    w_current   = 0.0;
    w_left      = 0.0;
    w_right     = 0.0;

    t_step      = 0.01;

    v_wheel_current = 0.0;
    w_wheel_current = 0.0;


    initOdom();
}



CarModel::~CarModel(){}


Status CarModel::checkLimit(double vw_target[2]){
    /*
    用于检查指令速度是否满足硬件限速
    线速度超标 0x1301
    角速度超标 0x1302
    */
    // cout << "[MODEL] Checking velocity..." << endl;
    // cout << "\t\tv_car = " << vw_target[0] << "  w_car = " << vw_target[1] << endl;

    double& v_target = vw_target[0];
    double& w_target = vw_target[1];
    if(v_target > v_ulimit) {
        // cout << "[MODEL] Illegal linear velocity! V overflow." << endl;
        return 0x00030101;
    }
    if(w_target > w_ulimit) {
        // cout << "[MODEL] Illegal rotate velocity! W overflow." << endl;
        return 0x00030102;
    }
    if(v_target < v_llimit) {
        // cout << "[MODEL] Illegal linear velocity! V underflow." << endl;
        return 0x00030101;
    }
    if(w_target < w_llimit) {
        // cout << "[MODEL] Illegal rotate velocity! W underflow." << endl;
        return 0x00030102;
    }
    // cout << "[MODEL] Velocity legal!" << endl;
    return CAR_SUCCESS;
}




void CarModel::carVtoMotorW(double vw_car[2], double w_motor[2]){
    /*
    vw_car[2] = {v_car, w_car}
    w_motor[2] = {w_motor_left, w_motor_right}
    */

    w_motor[0] = (vw_car[0] - 0.5 * l_tread * vw_car[1]) / r_wheel;
    w_motor[1] = (vw_car[0] + 0.5 * l_tread * vw_car[1]) / r_wheel;

}



void CarModel::motorWtoCarV(double w_motor[2], double vw_car[2]){
    /*
    w_motor[2] = {w_motor_left, w_motor_right}
    vw_car[2] = {v_car, w_car}
    */
    vw_car[0] = 0.5 * r_wheel * (w_motor[0] + w_motor[1]);
    vw_car[1] = r_wheel * (- w_motor[0] + w_motor[1]) / l_tread;

}


void CarModel::updateVelocity(double vw_target[2]){
    /*
    设置新速度，同时刷新里程计信息
    */
    //cout << "[MODEL] Set new velocity as";
    // cout << " v_car = " << vw_target[0] << "  w_car = " << vw_target[1] << endl;

    _updateOdom();
    v_current = vw_target[0];
    w_current = vw_target[1];
}



void CarModel::initOdom(){
    /*
    里程计位姿初始化（全零）
    */

    // 指令里程计
    odomX_last      = 0.0;
    odomY_last      = 0.0;
    odomPhi_last    = 0.0;
    auto    t_chrono    = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());
    int64_t t_last      = t_chrono.count();

    // 轮式里程计
    odomX_wheel     = 0.0;
    odomY_wheel     = 0.0;
    odomPhi_wheel   = 0.0;
}



void CarModel::resetOdom(double odom_pose[3]){
    /*
    里程计位姿重设
    */

    // 指令里程计
    odomX_last      = odom_pose[0];
    odomY_last      = odom_pose[1];
    odomPhi_last    = odom_pose[2];
    auto    t_chrono    = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());
    int64_t t_last      = t_chrono.count();

    // 轮式里程计
    odomX_wheel     = odom_pose[0];
    odomY_wheel     = odom_pose[1];
    odomPhi_wheel   = odom_pose[2];
}


void CarModel::getOrderOdom(double odom[5]){
    /*
    * 获取命令里程计（包含位姿和速度，只做计算不做更新），在收到获取命令里程计的指令后调用
    * odom[5] = {x, y, z, v, w}
    * 使用公式：
    * w_0 != 0:
    *     x_t     = x_0 - v_0 / w_0 * (cos(phi_0) - cos(phi_0 + w_0 * t))
    *     y_t     = y_0 - v_0 / w_0 * (sin(phi_0) - sin(phi_0 + w_0 * t))
    *     phi_t   = phi_0 + w_0 * t
    * 
    * w_0 == 0:
    *     x_t     = x_0 - v_0 * t * sin(phi_0)
    *     y_t     = y_0 + v_0 * t * cos(phi_0)
    *    phi_t   = phi_0
    */
    
    auto    t_chrono    = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());
    int64_t t_current   = t_chrono.count();
    double  t_interval  = (t_current - t_last) / 1000.0;

    if(w_current == 0.0){
        odom[0] = odomX_last - v_current * t_interval * sin(odomPhi_last);
        odom[1] = odomY_last + v_current * t_interval * cos(odomPhi_last);
    } else{
        odom[0] = odomX_last - v_current / w_current * (cos(odomPhi_last) - cos(odomPhi_last + w_current * t_interval));
        odom[1] = odomY_last - v_current / w_current * (sin(odomPhi_last) - sin(odomPhi_last + w_current * t_interval));
    }
    odom[2] = odomPhi_last + t_interval * w_current;
    odom[2] = atan2(sin(odom[2]), cos(odom[2]));
    odom[3] = v_current;
    odom[4] = w_current;
}


void CarModel::_updateOdom(){
    /*
    刷新里程计的位姿（计算完毕后更新系统内的位姿值），在收到新的速度指令后调用
    使用公式：
    w_0 != 0:
        x_t     = x_0 - v_0 / w_0 * (cos(phi_0) - cos(phi_0 + w_0 * t))
        y_t     = y_0 - v_0 / w_0 * (sin(phi_0) - sin(phi_0 + w_0 * t))
        phi_t   = phi_0 + w_0 * t

    w_0 == 0:
        x_t     = x_0 - v_0 * t * sin(phi_0)
        y_t     = y_0 + v_0 * t * cos(phi_0)
        phi_t   = phi_0
    */

    auto    t_chrono    = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());
    int64_t t_current   = t_chrono.count();
    double  t_interval  = (t_current - t_last) / 1000.0;

    if(w_current == 0.0){
        odomX_last  = odomX_last - v_current * t_interval * sin(odomPhi_last);
        odomY_last  = odomY_last + v_current * t_interval * cos(odomPhi_last);
    } else{
        odomX_last  = odomX_last - v_current / w_current * (cos(odomPhi_last) - cos(odomPhi_last + w_current * t_interval));
        odomY_last  = odomY_last - v_current / w_current * (sin(odomPhi_last) - sin(odomPhi_last + w_current * t_interval));
        odomPhi_last = odomPhi_last + t_interval * w_current;
        odomPhi_last = atan2(sin(odomPhi_last), cos(odomPhi_last));
    }
    t_last = t_current;
}


void CarModel::getWheelOdom(double odom[5]){
    /*
    * 获取轮式里程计
    */

    odom[0] = odomX_wheel;
    odom[1] = odomY_wheel;
    odom[2] = odomPhi_wheel;
    odom[3] = v_wheel_current;
    odom[4] = w_wheel_current;
}

void CarModel::updateWheelOdom(double vw_car[2]){
    /*
    * 刷新轮式里程计
    */
    odomX_wheel = odomX_wheel - v_wheel_current * t_step * sin(odomPhi_wheel);
    odomY_wheel = odomY_wheel + v_wheel_current * t_step * cos(odomPhi_wheel);
    odomPhi_wheel = odomPhi_wheel + t_step * w_wheel_current;
    odomPhi_wheel = atan2(sin(odomPhi_wheel), cos(odomPhi_wheel));
    v_wheel_current = vw_car[0];
    w_wheel_current = vw_car[1];
}