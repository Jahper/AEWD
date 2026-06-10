#ifndef IMU_PUBLISHER_H
#define IMU_PUBLISHER_H

#include "sensor_msgs/msg/imu.h"
class ImuPublisher
{
    public:
        void init(int baud);
        void update_msg(sensor_msgs__msg__Imu* msg);
        void step();
    private:

    
};

#endif