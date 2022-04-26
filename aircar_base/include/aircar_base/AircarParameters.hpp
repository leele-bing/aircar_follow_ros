#include"Common.h"
#include"Car.h"

using namespace std;



struct CarModelParameter carModelParameter = {
    wheel_radiu                 : 0.072,
    tread_length                : 0.56,
    linear_velocity_up_limit    : 3.0,
    rotate_velocity_up_limit    : 3.1415926,
    linear_velocity_low_limit   : -3.0,
    rotate_velocity_low_limit   : -3.1415926
};



struct MotorParameter motorParameter = {
    gear_rate       : 25,
    rad_code_rate   : 8192.0 / 100.0 / 3.14159265,
    left_sign       : -1,
    right_sign      : 1,
    left_motor_id   : 0x01,
    right_motor_id  : 0x02
};
