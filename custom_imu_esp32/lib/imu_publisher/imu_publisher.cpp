#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/timing.h>

#include "sensor_msgs/msg/imu.h"

#include "imu_publisher.hpp"


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rcl_publisher_t publisher;
const char * topic_name = "imu";
sensor_msgs__msg__Imu* imu_msg;

rclc_executor_t executor;
const unsigned int max_executor_timeout = 10;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
const char * node_name = "imu_node";
rcl_timer_t timer;
const unsigned int timer_timeout = 50;
int flash_count = 0;



//Used to debug ros initialization (using print via serial interferes with ros messages)
void error_flash(int count){
  for(int i = 0; i < count; i++){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}

// Error handle loop
void error_loop() {
  while(1) {
    error_flash(flash_count);
    delay(2000);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, imu_msg, NULL));
  }
}


void ImuPublisher::init(int baud){
    // Configure serial transport
    Serial.begin(baud);
    pinMode(LED_BUILTIN, OUTPUT);
    delay(1000);
    set_microros_serial_transports(Serial);
    delay(1000);
    rmw_uros_sync_session(1000);
    digitalWrite(LED_BUILTIN, HIGH);
    allocator = rcl_get_default_allocator();

    //create init_options
    flash_count ++; // 1
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    flash_count ++; // 2
    RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

    // create publisher
    flash_count ++; // 3
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        topic_name));

    // create timer,
    flash_count ++; // 4
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // create executor
    flash_count ++; // 5
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    flash_count ++; // 6
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
}


void ImuPublisher::update_msg(sensor_msgs__msg__Imu* msg){
    imu_msg = msg;
    int64_t now_ns = rmw_uros_epoch_nanos();
    imu_msg->header.frame_id.data = "bno055";
    imu_msg->header.stamp.sec = (int32_t)(now_ns/1000000000ULL);
    imu_msg->header.stamp.nanosec = (int32_t)(now_ns % 1000000000ULL);
}

void ImuPublisher::step(){
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

