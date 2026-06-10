#ifndef IMU_READER_H
#define IMU_READER_H

#include "BNO055_support.h"

struct ImuData
{
    unsigned long lastTime = 0;
    //quaternion
    double qw = 0;
    double qx = 0;
    double qy = 0;
    double qz = 0;

    //linear acceleration
    double lax = 0;
    double lay = 0;
    double laz = 0;

    // angular velocity
    double avx = 0;
    double avy = 0;
    double avz = 0;


};

class ImuReader
{
public:
    void init();
    void read_imu_data();
    ImuData get_data();
private:
    struct bno055_t BNO;
    struct bno055_quaternion quaternion;
    struct bno055_accel accel;
    struct bno055_gyro gyro;
    const float quat_scalar = 16384;
    
    ImuData data;
    
    // calibration vars
    unsigned char accelCalibStatus = 0;
    unsigned char magCalibStatus = 0;
    unsigned char gyroCalibStatus = 0;
    unsigned char sysCalibStatus = 0;
    
};
  

#endif