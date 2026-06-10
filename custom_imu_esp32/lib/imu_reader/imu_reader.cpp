#include <Wire.h>
#include "BNO055_support.h"
#include "imu_reader.hpp"


void ImuReader::init(){
    Wire.begin();
    BNO_Init(&BNO);
    // sets imu to NDoF (nine degrees of freedom)
    bno055_set_operation_mode(OPERATION_MODE_NDOF);
    delay(1);
};


void ImuReader::read_imu_data(){
    bno055_read_quaternion_wxyz(&quaternion);
    bno055_read_accel_xyz(&accel);
    bno055_read_gyro_xyz(&gyro);
    data.lastTime = millis();
    data.qw = quaternion.w / quat_scalar;
    data.qx = quaternion.x / quat_scalar;
    data.qy = quaternion.y / quat_scalar;
    data.qz = quaternion.z / quat_scalar;

    data.lax = accel.x;
    data.lay = accel.y;
    data.laz = accel.z;

    data.avx = gyro.x;
    data.avy = gyro.y;
    data.avz = gyro.z;
};

ImuData ImuReader::get_data(){
    return data;
}
