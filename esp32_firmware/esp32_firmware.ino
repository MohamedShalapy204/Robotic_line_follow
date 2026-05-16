#include <micro_ros_arduino.h>
#include <stdio.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// --- WI-FI & MICRO-ROS AGENT CONFIGURATION ---
char ssid[] = "ronin";               // اسم شبكة الواي فاي
char psk[] = "ronin1234";            // كلمة سر الواي فاي
char agent_ip[] = "192.168.137.206";  // تم التعديل لنص بين علامتي تنصيص  // IP جهاز الـ ROS2 الخاص بكِ
size_t agent_port = 8888;

// --- PIN DEFINITIONS ---
#define PIN_IR_L2 36
#define PIN_IR_L1 39
#define PIN_IR_MID 34
#define PIN_IR_R1 35
#define PIN_IR_R2 32

#define PIN_L_ENA 14
#define PIN_L_IN1 18
#define PIN_L_IN2 19
#define PIN_R_ENB 12
#define PIN_R_IN3 22
#define PIN_R_IN4 23

#define PIN_ENC_L 33
#define PIN_ENC_R 25
#define PIN_LED 13

// --- PWM SETTINGS ---
const int pwm_freq = 5000;
const int pwm_res = 8;

// --- ROS ENTITIES ---
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_publisher_t pub_ir[5];
rcl_publisher_t pub_enc_l, pub_enc_r;
rcl_subscription_t sub_motor_l, sub_motor_r;

std_msgs__msg__Int32 msg_ir[5];
std_msgs__msg__Int32 msg_enc_l, msg_enc_r;
std_msgs__msg__Int32 msg_motor_l, msg_motor_r;

// --- STATE ---
volatile long enc_l_ticks = 0;
volatile long enc_r_ticks = 0;
unsigned long last_cmd_time = 0;
unsigned long last_pub_time = 0;
const unsigned long timeout_ms = 500;

// --- MACROS ---
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }

void error_loop() {
  while (1) {
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    delay(100);
  }
}

// دالات المقاطعة (Interrupts) لحساب نبضات الإنكودر
void IRAM_ATTR count_l() {
  enc_l_ticks++;
}
void IRAM_ATTR count_r() {
  enc_r_ticks++;
}

void set_motor_speed(int pin_en, int in1, int in2, int pwm) {
  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pin_en, pwm);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pin_en, -pwm);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pin_en, 0);
  }
}

void sub_motor_l_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  set_motor_speed(PIN_L_ENA, PIN_L_IN1, PIN_L_IN2, msg->data);
  last_cmd_time = millis();
}

void sub_motor_r_callback(const void *msgin) {
  // تم حذف السطر الخاطئ الذي كان يسبب مشكلة في الكومبايل
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  set_motor_speed(PIN_R_ENB, PIN_R_IN3, PIN_R_IN4, msg->data);
  last_cmd_time = millis();
}

void setup() {
  pinMode(PIN_LED, OUTPUT);

  // 1. الاتصال بالـ Wi-Fi أولاً وضمان استقرار الاتصال
  WiFi.begin(ssid, psk);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));  // وميض سريع أثناء محاولة الاتصال
  }
  digitalWrite(PIN_LED, LOW);  // إطفاء الليد بعد نجاح الاتصال

  // إعداد الـ micro-ROS ليستخدم الواي فاي
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

  // IR Sensors
  pinMode(PIN_IR_L2, INPUT);
  pinMode(PIN_IR_L1, INPUT);
  pinMode(PIN_IR_MID, INPUT);
  pinMode(PIN_IR_R1, INPUT);
  pinMode(PIN_IR_R2, INPUT);

  // Motors
  pinMode(PIN_L_IN1, OUTPUT);
  pinMode(PIN_L_IN2, OUTPUT);
  pinMode(PIN_R_IN3, OUTPUT);
  pinMode(PIN_R_IN4, OUTPUT);
  ledcAttach(PIN_L_ENA, pwm_freq, pwm_res);
  ledcAttach(PIN_R_ENB, pwm_freq, pwm_res);

  // Encoders (تفعيل المقاطعات لقراءة الحساسات بدقة)
  pinMode(PIN_ENC_L, INPUT_PULLUP);
  pinMode(PIN_ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L), count_l, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R), count_r, RISING);

  delay(2000);
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_bridge", "", &support));

  // Publishers
  const char *ir_topics[] = { "raw/sensor_l2", "raw/sensor_l1", "raw/sensor_mid", "raw/sensor_r1", "raw/sensor_r2" };
  for (int i = 0; i < 5; i++) {
    RCCHECK(rclc_publisher_init_default(&pub_ir[i], &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), ir_topics[i]));
  }
  RCCHECK(rclc_publisher_init_default(&pub_enc_l, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "raw/encoder_l"));
  RCCHECK(rclc_publisher_init_default(&pub_enc_r, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "raw/encoder_r"));

  // Subscribers
  RCCHECK(rclc_subscription_init_default(&sub_motor_l, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor/left/pwm"));
  RCCHECK(rclc_subscription_init_default(&sub_motor_r, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor/right/pwm"));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor_l, &msg_motor_l, &sub_motor_l_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor_r, &msg_motor_r, &sub_motor_r_callback, ON_NEW_DATA));

  digitalWrite(PIN_LED, HIGH);  // إضاءة ثابتة تعني أن كل شيء يعمل ومرتبط بالـ Agent
}

void loop() {
  // Throttle publishing to 20Hz (every 50ms)
  if (millis() - last_pub_time > 50) {
    last_pub_time += 50;

    // نشر قراءات الـ IR
    int ir_pins[] = { PIN_IR_L2, PIN_IR_L1, PIN_IR_MID, PIN_IR_R1, PIN_IR_R2 };
    for (int i = 0; i < 5; i++) {
      msg_ir[i].data = analogRead(ir_pins[i]);
      RCSOFTCHECK(rcl_publish(&pub_ir[i], &msg_ir[i], NULL));
    }

    // حماية قراءة الـ Encoders من التداخل أثناء حدوث المقاطعة (Interrupt)
    noInterrupts();
    long current_enc_l = enc_l_ticks;
    long current_enc_r = enc_r_ticks;
    interrupts();

    // نشر قراءات الـ Encoders إلى الـ ROS2
    msg_enc_l.data = current_enc_l;
    msg_enc_r.data = current_enc_r;
    RCSOFTCHECK(rcl_publish(&pub_enc_l, &msg_enc_l, NULL));
    RCSOFTCHECK(rcl_publish(&pub_enc_r, &msg_enc_r, NULL));
  }

  // Failsafe (إيقاف المحركات إذا انقطع الاتصال)
  if (millis() - last_cmd_time > timeout_ms) {
    set_motor_speed(PIN_L_ENA, PIN_L_IN1, PIN_L_IN2, 0);
    set_motor_speed(PIN_R_ENB, PIN_R_IN3, PIN_R_IN4, 0);
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
}