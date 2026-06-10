#include <Arduino.h>
#include <Wire.h>

#include "BNO055_support.h"

#include "imu_reader.hpp"
#include "imu_publisher.hpp"
#include "sensor_msgs/msg/imu.h"

const int BAUD = 115200;
const int UPDATE_DELAY_MS = 10;
bool isCalibrated = true; // set false if you want to wait for full calibration 
unsigned long lastTime = 0;

struct bno055_t BNO;
struct bno055_euler eulerData;

// calibration vars
unsigned char accelCalibStatus = 0;
unsigned char magCalibStatus = 0;
unsigned char gyroCalibStatus = 0;
unsigned char sysCalibStatus = 0;

//functions
void get_i2c_devices();
void read_bno_information();
bool calibrate_imu();
void read_imu_data();


void fill_ros_msg();
void print_imu_data();

ImuReader reader;
ImuPublisher imu_publisher;
sensor_msgs__msg__Imu msg;

void setup() 
{
  reader.init();
  imu_publisher.init(BAUD);
  fill_ros_msg();
}

void loop() {
  // put your main code here, to run repeatedly:
   if(millis() - lastTime >= UPDATE_DELAY_MS)
	{
    lastTime = millis();
    reader.read_imu_data();
    fill_ros_msg();
    imu_publisher.update_msg(&msg);
    imu_publisher.step();
    // print_imu_data();
  }
}

void fill_ros_msg()
{
    ImuData data = reader.get_data();
    msg.orientation.x = data.qx;
    msg.orientation.y = data.qy;
    msg.orientation.z = data.qz;
    msg.orientation.w = data.qw;

    
    msg.angular_velocity.x = data.avx;
    msg.angular_velocity.y = data.avy;
    msg.angular_velocity.z = data.avz;   
    
    msg.linear_acceleration.x = data.lax;
    msg.linear_acceleration.y = data.lay;
    msg.linear_acceleration.z = data.laz;
}

//quick debug to see if imu reads data
void print_imu_data(){
  if(!Serial) Serial.begin(BAUD);
  ImuData data = reader.get_data();
  Serial.println("--- DATA ---");
  Serial.print("lastTime: "); Serial.println(data.lastTime);

  Serial.print("qw: "); Serial.println(data.qw);
  Serial.print("qx: "); Serial.println(data.qx);
  Serial.print("qy: "); Serial.println(data.qy);
  Serial.print("qz: "); Serial.println(data.qz);

  Serial.print("lax: "); Serial.println(data.lax);
  Serial.print("lay: "); Serial.println(data.lay);
  Serial.print("laz: "); Serial.println(data.laz);

  Serial.print("avx: "); Serial.println(data.avx);
  Serial.print("avy: "); Serial.println(data.avy);
  Serial.print("avz: "); Serial.println(data.avz);
}







