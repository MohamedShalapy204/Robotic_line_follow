#include <micro_ros_arduino.h>
#include <stdio.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>

// --- WI-FI & MICRO-ROS AGENT CONFIGURATION ---
char ssid[] = "wafaa hammad";              
char psk[] = "12345678";   
char agent_ip[] = "192.168.137.88"; 
size_t agent_port = 8888;   

// --- PIN DEFINITIONS ---
#define PIN_IR_L2 36
#define PIN_IR_L1 39
#define PIN_IR_MID 34
#define PIN_IR_R1 35
#define PIN_IR_R2 32

// --- دبابيس الشيلد الأزرق (L293D Shift Register Pins) ---
#define PIN_M1_PWM 14  // سرعة الموتور اليسار (M1)
#define PIN_M2_PWM 27  // سرعة الموتور اليمين (M2)
#define PIN_LATCH 18   // خط القفل
#define PIN_CLK 19     // خط النبضات
#define PIN_DATA 23    // خط البيانات
#define PIN_EN 22      // خط تفعيل الشريحة (Active Low)

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
rcl_publisher_t pub_pwm_l, pub_pwm_r;  // نشر قيم الـ PWM الحقيقية للـ GUI
rcl_subscription_t sub_cmd_vel;         // استقبال أوامر الحركة مباشرة

std_msgs__msg__Int32 msg_ir[5];
std_msgs__msg__Int32 msg_enc_l, msg_enc_r;
std_msgs__msg__Int32 msg_pwm_l, msg_pwm_r;
geometry_msgs__msg__Twist msg_cmd_vel;

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

// متغير لحفظ حالة اتجاه المواتير في الـ Shift Register
uint8_t latch_state = 0;

// دالة لإرسال الأوامر لشريحة الـ Shift Register
void updateShiftRegister() {
  digitalWrite(PIN_LATCH, LOW);
  shiftOut(PIN_DATA, PIN_CLK, MSBFIRST, latch_state);
  digitalWrite(PIN_LATCH, HIGH);
}

// الدالة الجديدة للتحكم في المواتير (رقم 1 لليسار، رقم 2 لليمين)
void set_motor_speed(int motor, int pwm) {
  int pin_pwm;
  uint8_t bit_A, bit_B;

  // تحديد البتات الخاصة بكل موتور حسب تصميم شيلد L293D
  if (motor == 1) { // الموتور اليسار (M1)
    pin_pwm = PIN_M1_PWM;
    bit_A = 4; // Bit 2
    bit_B = 8; // Bit 3
  } else {          // الموتور اليمين (M2)
    pin_pwm = PIN_M2_PWM;
    bit_A = 2;  // Bit 1
    bit_B = 16; // Bit 4
  }

  // تحديد الاتجاه بناءً على قيمة الـ PWM (معكوسة لتصحيح الاتجاه)
  if (pwm > 0) {
    latch_state &= ~bit_A;  // أمامي (معدّل)
    latch_state |= bit_B;
    ledcWrite(pin_pwm, pwm);
  } else if (pwm < 0) {
    latch_state |= bit_A;   // خلفي (معدّل)
    latch_state &= ~bit_B;
    ledcWrite(pin_pwm, -pwm);
  } else {
    latch_state &= ~bit_A;  // توقف
    latch_state &= ~bit_B;
    ledcWrite(pin_pwm, 0);
  }
  
  // تطبيق الاتجاه فوراً
  updateShiftRegister();
}

// ==========================================
// --- IMPROVED VELOCITY COMMAND CALLBACK ---
// ==========================================
void sub_cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // ROS 2 Twist messages use 'double' precision
  double v = msg->linear.x;
  double omega = msg->angular.z;
  
  // Wheel kinematics
  float wheel_separation = 0.135;
  float l_raw = v - (omega * wheel_separation / 2.0);
  float r_raw = v + (omega * wheel_separation / 2.0);

  // Convert to initial PWM
  int pwm_l = (int)(l_raw * 255.0);
  int pwm_r = (int)(r_raw * 255.0);
  
  // Clamp to absolute hardware limits first
  if (pwm_l > 255) pwm_l = 255;
  if (pwm_l < -255) pwm_l = -255;
  if (pwm_r > 255) pwm_r = 255;
  if (pwm_r < -255) pwm_r = -255;
  
  // Deadband mapping (L293D requires ~90 PWM to move)
  int min_pwm = 90;
  
  // MAP the values instead of clamping them. 
  // This preserves the PID steering resolution below the deadband limit.
  if (pwm_l > 0) {
    pwm_l = map(pwm_l, 1, 255, min_pwm, 255);
  } else if (pwm_l < 0) {
    pwm_l = map(pwm_l, -1, -255, -min_pwm, -255);
  }
  
  if (pwm_r > 0) {
    pwm_r = map(pwm_r, 1, 255, min_pwm, 255);
  } else if (pwm_r < 0) {
    pwm_r = map(pwm_r, -1, -255, -min_pwm, -255);
  }
  
  // Drive the physical motors
  set_motor_speed(1, pwm_l);
  set_motor_speed(2, pwm_r);
  
  // Publish actual PWM values for Web GUI monitoring
  msg_pwm_l.data = pwm_l;
  msg_pwm_r.data = pwm_r;
  RCSOFTCHECK(rcl_publish(&pub_pwm_l, &msg_pwm_l, NULL));
  RCSOFTCHECK(rcl_publish(&pub_pwm_r, &msg_pwm_r, NULL));
  
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

  // إعداد دبابيس الشيلد الجديد
  pinMode(PIN_LATCH, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  
  digitalWrite(PIN_EN, LOW); // تفعيل الشريحة (Active Low)
  updateShiftRegister();     // التأكد من توقف المواتير عند بدء التشغيل

  ledcAttach(PIN_M1_PWM, pwm_freq, pwm_res);
  ledcAttach(PIN_M2_PWM, pwm_freq, pwm_res);

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
  RCCHECK(rclc_publisher_init_default(&pub_pwm_l, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor/left/pwm"));
  RCCHECK(rclc_publisher_init_default(&pub_pwm_r, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor/right/pwm"));

  // Subscribers
  RCCHECK(rclc_subscription_init_default(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &sub_cmd_vel_callback, ON_NEW_DATA));

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
    set_motor_speed(1, 0);
    set_motor_speed(2, 0);
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
}