#include <micro_ros_arduino.h>
#include <stdio.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
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

// مصفوفة دبابيس الحساسات (تم نقلها لتوفير الذاكرة)
const int ir_pins[] = { PIN_IR_L2, PIN_IR_L1, PIN_IR_MID, PIN_IR_R1, PIN_IR_R2 };

// --- PWM SETTINGS ---
const int pwm_freq = 5000;
const int pwm_res = 8;

// --- ROS ENTITIES ---
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_publisher_t pub_enc_l, pub_enc_r;
rcl_publisher_t pub_pwm_l, pub_pwm_r;  // نشر قيم الـ PWM الحقيقية للـ GUI
rcl_publisher_t pub_error;              // نشر قيمة الـ line_error
rcl_publisher_t pub_mission;            // نشر أوامر الـ mission_control
rcl_publisher_t pub_cmd_vel;            // نشر أوامر الحركة للمراقبة
rcl_subscription_t sub_cmd_vel;         // استقبال أوامر الحركة مباشرة
rcl_subscription_t sub_tuning;          // استقبال معاملات الضبط
rcl_subscription_t sub_mission;         // استقبال أوامر التشغيل/الإيقاف

std_msgs__msg__Int32 msg_enc_l, msg_enc_r;
std_msgs__msg__Int32 msg_pwm_l, msg_pwm_r;
std_msgs__msg__Float32 msg_error;
std_msgs__msg__String msg_mission;
std_msgs__msg__String msg_mission_sub;
char mission_sub_buffer[32];
std_msgs__msg__String msg_tuning;
char tuning_buffer[128];
geometry_msgs__msg__Twist msg_cmd_vel;

// --- STATE ---
volatile long enc_l_ticks = 0;
volatile long enc_r_ticks = 0;
unsigned long last_cmd_time = 0;
unsigned long last_pub_time = 0;
const unsigned long timeout_ms = 500;

// --- LINE SENSOR & PID STATE ---
int hw_threshold = 2000;
float last_error = 0.0;
unsigned long line_lost_start_time = 0;
bool stopped_by_line_loss = false;

bool is_active = false;
float kp = 1.2;
float ki = 0.0;
float kd = 0.1;
float base_speed = 0.3;
float prev_error = 0.0;
float integral = 0.0;
int kickstart_count = 0;
bool kickstart_enabled = false;
float wheel_separation = 0.135;

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

  // تحديد الاتجاه بناءً على قيمة الـ PWM
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
// --- IMPROVED TUNING & VELOCITY CALLBACKS ---
// ==========================================
float get_json_float(String data, String key, float default_val) {
  int index = data.indexOf(key);
  if (index != -1) {
    int colon = data.indexOf(':', index);
    if (colon != -1) {
      int comma = data.indexOf(',', colon);
      if (comma == -1) comma = data.indexOf('}', colon);
      if (comma != -1) {
        String val_str = data.substring(colon + 1, comma);
        val_str.trim();
        return val_str.toFloat();
      }
    }
  }
  return default_val;
}

bool get_json_bool(String data, String key, bool default_val) {
  int index = data.indexOf(key);
  if (index != -1) {
    int colon = data.indexOf(':', index);
    if (colon != -1) {
      int comma = data.indexOf(',', colon);
      if (comma == -1) comma = data.indexOf('}', colon);
      if (comma != -1) {
        String val_str = data.substring(colon + 1, comma);
        val_str.trim();
        return val_str.equals("true");
      }
    }
  }
  return default_val;
}

void sub_tuning_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  
  // Safe null-termination of deserialized buffer
  int len = msg->data.size;
  if (len >= 128) len = 127;
  tuning_buffer[len] = '\0';
  
  String data = String(tuning_buffer);
  
  hw_threshold = (int)get_json_float(data, "sensor_threshold", hw_threshold);
  kp = get_json_float(data, "kp", kp);
  ki = get_json_float(data, "ki", ki);
  kd = get_json_float(data, "kd", kd);
  base_speed = get_json_float(data, "base_speed", base_speed);
  kickstart_enabled = get_json_bool(data, "kickstart_enabled", kickstart_enabled);
}

void sub_mission_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  
  // Safe null-termination of deserialized buffer
  int len = msg->data.size;
  if (len >= 32) len = 31;
  mission_sub_buffer[len] = '\0';
  
  String command = String(mission_sub_buffer);
  command.toLowerCase();
  command.trim();
  
  if (command.equals("start")) {
    is_active = true;
    prev_error = 0.0;
    integral = 0.0;
    if (kickstart_enabled) {
      kickstart_count = 5;
    }
  } else if (command.equals("stop")) {
    is_active = false;
    set_motor_speed(1, 0);
    set_motor_speed(2, 0);
  }
}

void sub_cmd_vel_callback(const void *msgin) {
  if (is_active) return; // Ignore manual velocity commands while in Autonomous mode
  
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // ROS 2 Twist messages use 'double' precision
  double v = msg->linear.x;
  double omega = msg->angular.z;
  
  // Wheel kinematics (Flipped signs to fix reversed steering)
  float l_raw = v + (omega * wheel_separation / 2.0);
  float r_raw = v - (omega * wheel_separation / 2.0);

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
  RCCHECK(rclc_publisher_init_default(&pub_enc_l, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "raw/encoder_l"));
  RCCHECK(rclc_publisher_init_default(&pub_enc_r, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "raw/encoder_r"));
  RCCHECK(rclc_publisher_init_default(&pub_pwm_l, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor/left/pwm"));
  RCCHECK(rclc_publisher_init_default(&pub_pwm_r, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor/right/pwm"));
  RCCHECK(rclc_publisher_init_default(&pub_error, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "line_error"));
  RCCHECK(rclc_publisher_init_default(&pub_mission, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "mission_control"));
  RCCHECK(rclc_publisher_init_default(&pub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  // Subscribers
  RCCHECK(rclc_subscription_init_default(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_subscription_init_default(&sub_tuning, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "tuning_params"));
  RCCHECK(rclc_subscription_init_default(&sub_mission, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "mission_control"));

  // Configure string buffer for sub_tuning & sub_mission
  msg_tuning.data.data = tuning_buffer;
  msg_tuning.data.size = 0;
  msg_tuning.data.capacity = 128;

  msg_mission_sub.data.data = mission_sub_buffer;
  msg_mission_sub.data.size = 0;
  msg_mission_sub.data.capacity = 32;

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &sub_cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_tuning, &msg_tuning, &sub_tuning_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_mission, &msg_mission_sub, &sub_mission_callback, ON_NEW_DATA));

  digitalWrite(PIN_LED, HIGH);  // إضاءة ثابتة تعني أن كل شيء يعمل ومرتبط بالـ Agent
}

void loop() {
  // Throttle publishing to 20Hz (every 50ms)
  if (millis() - last_pub_time > 50) {
    last_pub_time += 50;

    // 1. قراءة الحساسات وحساب الـ error
    float sensor_values[5];
    float total_on_line = 0;
    
    for (int i = 0; i < 5; i++) {
      int raw_val = analogRead(ir_pins[i]);
      
      // تحديد إذا كان فوق الخط أم لا (القراءة أصغر من العتبة تعني وجود خط)
      float is_line = (raw_val < hw_threshold) ? 1.0 : 0.0;
      sensor_values[i] = is_line;
      total_on_line += is_line;
    }
    
    // حساب انحراف الخط بالمعادلة الموزونة
    float weights[] = { 2.0, 1.0, 0.0, -1.0, -2.0 };
    float error = 0.0;
    
    if (total_on_line > 0) {
      line_lost_start_time = 0;
      stopped_by_line_loss = false;
      
      float sum_weighted = 0;
      for (int i = 0; i < 5; i++) {
        sum_weighted += sensor_values[i] * weights[i];
      }
      error = sum_weighted / total_on_line;
      last_error = error;
      
      msg_error.data = error;
      RCSOFTCHECK(rcl_publish(&pub_error, &msg_error, NULL));
    } else {
      // إذا فُقد الخط تماماً
      if (!stopped_by_line_loss) {
        if (line_lost_start_time == 0) {
          line_lost_start_time = millis();
        }
        
        unsigned long elapsed = millis() - line_lost_start_time;
        if (elapsed >= 500) {
          is_active = false;
          stopped_by_line_loss = true;
          set_motor_speed(1, 0);
          set_motor_speed(2, 0);
          
          // نشر أمر الإيقاف بعد نصف ثانية من فقدان الخط (استخدام static memory)
          static char stop_str[] = "stop";
          msg_mission.data.data = stop_str;
          msg_mission.data.size = strlen(stop_str);
          msg_mission.data.capacity = strlen(stop_str) + 1;
          RCSOFTCHECK(rcl_publish(&pub_mission, &msg_mission, NULL));
          
          msg_error.data = 0.0;
          RCSOFTCHECK(rcl_publish(&pub_error, &msg_error, NULL));
        } else {
          // فترة سماح: استمرار نشر آخر انحراف معروف
          error = last_error;
          msg_error.data = error;
          RCSOFTCHECK(rcl_publish(&pub_error, &msg_error, NULL));
        }
      }
    }

    // تشغيل نظام الـ PID الذاتي محلياً على الـ ESP32 إذا كان مفعلاً
    if (is_active && !stopped_by_line_loss) {
      integral += error;
      
      // Anti-windup
      float max_integral = 10.0;
      if (integral > max_integral) integral = max_integral;
      if (integral < -max_integral) integral = -max_integral;
      
      float derivative = error - prev_error;
      float angular_z = (kp * error) + (ki * integral) + (kd * derivative);
      prev_error = error;
      
      float linear_x = base_speed;
      if (kickstart_count > 0) {
        linear_x = 0.8;
        kickstart_count--;
      }
      
      // Wheel kinematics
      float l_raw = linear_x + (angular_z * wheel_separation / 2.0);
      float r_raw = linear_x - (angular_z * wheel_separation / 2.0);
      
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
      
      // Publish calculated Twist command back to ROS for telemetry monitoring
      msg_cmd_vel.linear.x = linear_x;
      msg_cmd_vel.angular.z = angular_z;
      RCSOFTCHECK(rcl_publish(&pub_cmd_vel, &msg_cmd_vel, NULL));
      
      // Publish actual PWM values for Web GUI monitoring
      msg_pwm_l.data = pwm_l;
      msg_pwm_r.data = pwm_r;
      RCSOFTCHECK(rcl_publish(&pub_pwm_l, &msg_pwm_l, NULL));
      RCSOFTCHECK(rcl_publish(&pub_pwm_r, &msg_pwm_r, NULL));
    }

    // 2. حماية قراءة الـ Encoders من التداخل أثناء المقاطعة
    noInterrupts();
    long current_enc_l = enc_l_ticks;
    long current_enc_r = enc_r_ticks;
    interrupts();

    // نشر قراءات الـ Encoders
    msg_enc_l.data = current_enc_l;
    msg_enc_r.data = current_enc_r;
    RCSOFTCHECK(rcl_publish(&pub_enc_l, &msg_enc_l, NULL));
    RCSOFTCHECK(rcl_publish(&pub_enc_r, &msg_enc_r, NULL));
  }

  // Failsafe (إيقاف المحركات إذا انقطع الاتصال في الوضع اليدوي)
  if (!is_active && (millis() - last_cmd_time > timeout_ms)) {
    set_motor_speed(1, 0);
    set_motor_speed(2, 0);
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
}