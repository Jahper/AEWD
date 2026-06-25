#include <Wire.h>
#include "BNO055_support.h"
#include "imu_reader.hpp"


void ImuReader::init(){
    Wire.begin();
    BNO_Init(&BNO);
    // sets imu to NDoF (nine degrees of freedom)
    bno055_set_operation_mode(OPERATION_MODE_NDOF);
    bno055_set_accel_unit(0); //set unit to m/s²
    bno055_set_gyro_unit(1); // set to rps
    delay(1);
};


void ImuReader::read_imu_data(){
    bno055_get_accelcalib_status(&calibstatus.accel);
    bno055_get_magcalib_status(&calibstatus.mag);
    bno055_get_gyrocalib_status(&calibstatus.gyro);
    bno055_get_syscalib_status(&calibstatus.sys);

    bno055_read_quaternion_wxyz(&quaternion);
    bno055_read_linear_accel_xyz(&accel);
    bno055_read_gyro_xyz(&gyro);
    
    
    data.lastTime = millis();
    data.qw = quaternion.w / QUAT_SCALAR;
    data.qx = quaternion.x / QUAT_SCALAR;
    data.qy = quaternion.y / QUAT_SCALAR;
    data.qz = quaternion.z / QUAT_SCALAR;

    data.lax = accel.x / ACCEL_SCALAR_MS2;
    data.lay = accel.y / ACCEL_SCALAR_MS2;
    data.laz = accel.z / ACCEL_SCALAR_MS2;

    data.avx = gyro.x / GYRO_SCALAR_RPS;
    data.avy = gyro.y / GYRO_SCALAR_RPS;
    data.avz = gyro.z / GYRO_SCALAR_RPS;
};

ImuData ImuReader::get_data(){
    return data;
};

CalibrationStatus ImuReader::get_calibration_status(){
    return calibstatus;

}
