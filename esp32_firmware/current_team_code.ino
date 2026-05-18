#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/quaternion.h>

// --- إعدادات شبكة الـ Wi-Fi و micro-ROS Agent ---
const char* ssid = "YOUR_WIFI_NAME";        // اسم شبكة الواي فاي
const char* password = "YOUR_WIFI_PASSWORD"; // باسورد الواي فاي
IPAddress agent_ip(192, 168, 1, 100);       // الـ IP بتاع اللاب توب بتاعك
const uint16_t agent_port = 8888;           // البورت الافتراضي لـ micro-ROS Agent

// --- كائنات micro-ROS ---
rcl_subscription_t subscriber;
geometry_msgs__msg__Quaternion msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){/* خطأ في الاتصال */} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){/* تحذير */} }

// --- PIN DEFINITIONS ---
#define PIN_IR_L2 36
#define PIN_IR_L1 39
#define PIN_IR_MID 34
#define PIN_IR_R1 35
#define PIN_IR_R2 32

#define PIN_M1_PWM 14  
#define PIN_M2_PWM 27  
#define PIN_LATCH 18   
#define PIN_CLK 19     
#define PIN_DATA 23    
#define PIN_EN 22      
#define PIN_LED 13

const int ir_pins[] = { PIN_IR_L2, PIN_IR_L1, PIN_IR_MID, PIN_IR_R1, PIN_IR_R2 };
const int pwm_freq = 5000;
const int pwm_res = 8;
int hw_threshold = 2000;  

// --- متغيرات الـ PID ---
float kp = 1.2;
float ki = 0.0;
float kd = 0.1;
float base_speed = 150.0; 

float prev_error = 0.0;
float integral = 0.0;   
float left_trim_factor = 1.145; 

// دالة تحديث قيم الـ PID اللي بيستدعيها الـ Agent
void tuneCallback(const void * msgin) {
  const geometry_msgs__msg__Quaternion * received_msg = (const geometry_msgs__msg__Quaternion *)msgin;
  kp = received_msg->x;           
  ki = received_msg->y;           
  kd = received_msg->z;           
  base_speed = received_msg->w;   
}

// --- متغيرات التحكم في الروبوت ---
enum State { FOLLOWING, STOPPED, ROTATING, FINISHED };
State robotState = FOLLOWING;
unsigned long stop_time = 0;
uint8_t latch_state = 0;
int lap_counter = 0;           
bool on_finish_line = false;   

void updateShiftRegister() {
  digitalWrite(PIN_LATCH, LOW);
  shiftOut(PIN_DATA, PIN_CLK, MSBFIRST, latch_state);
  digitalWrite(PIN_LATCH, HIGH);
}

void set_motor_speed(int motor, int pwm) {
  int pin_pwm;
  uint8_t bit_A, bit_B;

  if (motor == 1) pwm = (int)(pwm / left_trim_factor);

  if (pwm > 255) pwm = 255;
  if (pwm < -255) pwm = -255;

  if (motor == 1) { 
    pin_pwm = PIN_M1_PWM; bit_A = 4; bit_B = 8; 
  } else {          
    pin_pwm = PIN_M2_PWM; bit_A = 2; bit_B = 16; 
  }

  if (pwm > 0) {
    latch_state &= ~bit_A; latch_state |= bit_B;
    ledcWrite(pin_pwm, pwm);
  } else if (pwm < 0) {
    latch_state |= bit_A; latch_state &= ~bit_B;
    ledcWrite(pin_pwm, -pwm);
  } else {
    latch_state &= ~bit_A; latch_state &= ~bit_B;
    ledcWrite(pin_pwm, 0);
  }
  updateShiftRegister();
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  for (int i = 0; i < 5; i++) { pinMode(ir_pins[i], INPUT); }
  
  pinMode(PIN_LATCH, OUTPUT); pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_DATA, OUTPUT); pinMode(PIN_EN, OUTPUT);
  
  digitalWrite(PIN_EN, LOW); 
  updateShiftRegister();     

  ledcAttach(PIN_M1_PWM, pwm_freq, pwm_res);
  ledcAttach(PIN_M2_PWM, pwm_freq, pwm_res);
  digitalWrite(PIN_LED, LOW); 

  // --- إعداد اتصال micro-ROS عبر الواي فاي ---
  set_microros_wifi_transports((char*)ssid, (char*)password, agent_ip, agent_port);

  delay(2000); // ننتظر قليلاً للاتصال بالشبكة

  allocator = rcl_get_default_allocator();

  // تهيئة الـ Support والـ Node
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "pid_tuner_node", "", &support));

  // إنشاء الـ Subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
    "tune_pid"));

  // إعداد الـ Executor (المسؤول عن تنفيذ الـ Callbacks)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &tuneCallback, ON_NEW_DATA));
}

void loop() {
  // استقبال الأوامر من micro-ROS Agent (يتم تنفيذها إذا كان هناك بيانات جديدة)
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // قراءة الحساسات
  float sensor_values[5];
  float total_on_line = 0;
  for (int i = 0; i < 5; i++) {
    int raw_val = analogRead(ir_pins[i]);
    float is_line = (raw_val < hw_threshold) ? 1.0 : 0.0;
    sensor_values[i] = is_line;
    total_on_line += is_line;
  }

  switch (robotState) {
    case FOLLOWING:
      if (total_on_line >= 4) {
        if (on_finish_line == false) {
          lap_counter++;               // زود عداد اللفات
          on_finish_line = true;       // تأمين العداد لتسجيل لفة واحدة فقط في كل مرور
          if (lap_counter!=1){
            digitalWrite(PIN_LED, HIGH); // <-- التعديل هنا: نور الليد أول ما يلمس الخط
          }
        }
      } else {
        on_finish_line = false;        
        digitalWrite(PIN_LED, LOW);    
      }

      if (lap_counter >= 3) {
        set_motor_speed(1, 0);         
        set_motor_speed(2, 0);         
        digitalWrite(PIN_LED, HIGH);   
        robotState = FINISHED;         
      } 
      else if (total_on_line == 0) {
        set_motor_speed(1, 0);
        set_motor_speed(2, 0);
        stop_time = millis();
        robotState = STOPPED;
      } else {
        float weights[] = { 2.0, 1.0, 0.0, -1.0, -2.0 };
        float sum_weighted = 0;
        for (int i = 0; i < 5; i++) sum_weighted += sensor_values[i] * weights[i];
        float error = sum_weighted / total_on_line;

        // حسابات الـ PID
        integral += error; 
        if (integral > 50) integral = 50;
        if (integral < -50) integral = -50;

        float derivative = error - prev_error;
        float correction = (kp * error * 50) + (ki * integral * 50) + (kd * derivative * 50); 
        prev_error = error;

        int pwm_l = base_speed + correction;
        int pwm_r = base_speed - correction;

        int min_pwm = 90;
        if (pwm_l > 0 && pwm_l < min_pwm) pwm_l = min_pwm;
        if (pwm_r > 0 && pwm_r < min_pwm) pwm_r = min_pwm;

        set_motor_speed(1, pwm_l);
        set_motor_speed(2, pwm_r);
      }
      break;

    case STOPPED:
      if (millis() - stop_time > 1000) robotState = ROTATING;
      break;

    case ROTATING:
      set_motor_speed(1, -120); set_motor_speed(2, 120);
      if (sensor_values[2] > 0 || sensor_values[1] > 0 || sensor_values[3] > 0 ) {
        set_motor_speed(1, 0); set_motor_speed(2, 0);
        delay(200); prev_error = 0.0; integral = 0.0; 
        robotState = FOLLOWING;
      }
      break;

    case FINISHED:
      set_motor_speed(1, 0); set_motor_speed(2, 0);
      break;
  }
  // قللنا الـ Delay لأن الـ executor_spin_some بيعمل استهلاك للوقت شوية
  delay(5); 
}