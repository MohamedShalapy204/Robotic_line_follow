#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h> 


const int irPins[5] = {32, 33, 25, 26, 27};
const int enA = 14; 
const int enB = 12; 

const int motorRight1 = 22; // IN1
const int motorRight2 = 23; // IN2
const int motorLeft1 = 18;  // IN3
const int motorLeft2 = 19;  // IN4


const int freq = 30000;
const int pwmChannelA = 0; 
const int pwmChannelB = 1;
const int resolution = 8;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rcl_subscription_t subscriber;      
geometry_msgs__msg__Twist msg_sub; 
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    int sensor_value = 0;
    for(int i=0; i<5; i++) {
      if(digitalRead(irPins[i]) == HIGH) {
        sensor_value |= (1 << i);
      }
    }
    msg.data = sensor_value; 
    rcl_publish(&publisher, &msg, NULL);
  }
}

// ---
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg_in = (const geometry_msgs__msg__Twist *)msgin;
  
  float x = msg_in->linear.x;  
  float z = msg_in->angular.z; 


  int speed = (int)(abs(x) * 255); 


  if (speed > 0 && speed < 145) {
      speed = 145; 
  }

  if (x > 0) {
  
    ledcWrite(pwmChannelA, speed); 
    ledcWrite(pwmChannelB, speed);
    digitalWrite(motorRight1, HIGH); digitalWrite(motorRight2, LOW);
    digitalWrite(motorLeft1, HIGH); digitalWrite(motorLeft2, LOW);
  } 
  else if (z != 0) { 
    
    int spin_speed = (int)(abs(z) * 255);
    if (spin_speed < 145) spin_speed = 145;

    ledcWrite(pwmChannelA, spin_speed);
    ledcWrite(pwmChannelB, spin_speed);

    if (z > 0) { // دوران يسار
      digitalWrite(motorRight1, HIGH); digitalWrite(motorRight2, LOW);
      digitalWrite(motorLeft1, LOW); digitalWrite(motorLeft2, HIGH);
    } else { // دوران يمين
      digitalWrite(motorRight1, LOW); digitalWrite(motorRight2, HIGH);
      digitalWrite(motorLeft1, HIGH); digitalWrite(motorLeft2, LOW);
    }
  }
  else { 
    
    ledcWrite(pwmChannelA, 0);
    ledcWrite(pwmChannelB, 0);
    digitalWrite(motorRight1, LOW); digitalWrite(motorRight2, LOW);
    digitalWrite(motorLeft1, LOW); digitalWrite(motorLeft2, LOW);
  }
}

void setup() {
  set_microros_transports();

  for(int i=0; i<5; i++) { pinMode(irPins[i], INPUT); }
  pinMode(motorRight1, OUTPUT); pinMode(motorRight2, OUTPUT);
  pinMode(motorLeft1, OUTPUT); pinMode(motorLeft2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  

  ledcAttach(enA, freq, resolution);
  ledcAttach(enB, freq, resolution);

  allocator = rcl_get_default_allocator();


  while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }

  rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support);

  
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/line_sensors");


  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"); 


  const unsigned int timer_timeout = 50; // 
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback);

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA);

  digitalWrite(LED_PIN, HIGH); 
}

void loop() {
  delay(10);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}