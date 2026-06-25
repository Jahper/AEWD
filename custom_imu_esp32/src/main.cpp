#include <Arduino.h>
#include <Wire.h>

#include "BNO055_support.h"

#include "imu_reader.hpp"
#include "imu_publisher.hpp"
#include "sensor_msgs/msg/imu.h"

const int BAUD = 115200;
const int UPDATE_DELAY_MS = 10;
unsigned long lastTime = 0;
const bool DEBUG_IMU = false; //True if just want to test imu in isolation, false to run microros aswell.

ImuReader reader;
ImuPublisher imu_publisher;
sensor_msgs__msg__Imu msg;

//covariance
float cov_linear_accel;
float cov_angular_vel;


void fill_ros_msg();
void print_imu_data();
void print_imu_calib_status();


void setup() 
{
  reader.init();
  if(DEBUG_IMU)return;
  imu_publisher.init(BAUD);
  fill_ros_msg();
}

void loop() {
   if(millis() - lastTime >= UPDATE_DELAY_MS)
	{
    lastTime = millis();
    reader.read_imu_data();
    
    //sets covariance based on calibration
    bool imu_calibrated = (reader.get_calibration_status().sys == 3);
    cov_angular_vel = imu_calibrated ? 0.05f : 0.1f;
    cov_linear_accel = imu_calibrated ? 0.5f : 1.0f;

    if(!DEBUG_IMU){
      fill_ros_msg();
      imu_publisher.update_msg(&msg);
      imu_publisher.step();
    }else{
      print_imu_calib_status();
      print_imu_data();
    }
  }
}

void fill_ros_msg()
{
    ImuData data = reader.get_data();
    msg.orientation.x = data.qy;
    msg.orientation.y = data.qx;
    msg.orientation.z = -data.qz;
    msg.orientation.w = data.qw;

    msg.orientation_covariance[0] = 0.01;
    msg.orientation_covariance[4] = 0.01;
    msg.orientation_covariance[8] = 0.01;

    
    msg.angular_velocity.x = data.avx;
    msg.angular_velocity.y = data.avy;
    msg.angular_velocity.z = data.avz;   

    msg.angular_velocity_covariance[0] = cov_angular_vel;
    msg.angular_velocity_covariance[4] = cov_angular_vel;
    msg.angular_velocity_covariance[8] = cov_angular_vel;
    
    msg.linear_acceleration.x = data.lax;
    msg.linear_acceleration.y = data.lay;
    msg.linear_acceleration.z = data.laz;

    msg.linear_acceleration_covariance[0] = cov_linear_accel;
    msg.linear_acceleration_covariance[4] = cov_linear_accel;
    msg.linear_acceleration_covariance[8] = cov_linear_accel;
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

void print_imu_calib_status(){
  CalibrationStatus status = reader.get_calibration_status();
  Serial.print("Accel: ");
  Serial.print(status.accel);

  Serial.print(" | Mag: ");
  Serial.print(status.mag);

  Serial.print(" | Gyro: ");
  Serial.print(status.gyro);

  Serial.print(" | Sys: ");
  Serial.println(status.sys);
}






