#ifndef IMU_READER_H
#define IMU_READER_H

#include "BNO055_support.h"

struct ImuData
{
    unsigned long lastTime = 0;
    //quaternion
    float qw = 0;
    float qx = 0;
    float qy = 0;
    float qz = 0;

    //linear acceleration
    double lax = 0;
    double lay = 0;
    double laz = 0;

    // angular velocity
    double avx = 0;
    double avy = 0;
    double avz = 0;


};

struct CalibrationStatus{
    // calibration vars
    unsigned char accel = 0;
    unsigned char mag = 0;
    unsigned char gyro = 0;
    unsigned char sys = 0;
};

class ImuReader
{
public:
    void init();
    void read_imu_data();
    ImuData get_data();
    CalibrationStatus get_calibration_status();
private:
    struct bno055_t BNO;
    CalibrationStatus calibstatus;
    struct bno055_quaternion quaternion;
    struct bno055_linear_accel accel;
    struct bno055_gyro gyro;
    const float QUAT_SCALAR = 16384;
    const float ACCEL_SCALAR_MS2 = 100;
    const float GYRO_SCALAR_RPS = 900;
    
    ImuData data;
    
   
    
};
  

#endif