 #include <micro_ros_arduino.h>
#include <WiFi.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

// --- PIN DEFINITIONS (IR & Encoders - لم يتم تغييرها) ---
#define PIN_IR_L2 36
#define PIN_IR_L1 39
#define PIN_IR_MID 34
#define PIN_IR_R1 35
#define PIN_IR_R2 32

#define PIN_ENC_L 33
#define PIN_ENC_R 25
#define PIN_LED 13

// --- دبابيس الشيلد الأزرق (L293D Shift Register Pins) ---
#define PIN_M1_PWM 14  // سرعة الموتور اليسار (M1)
#define PIN_M2_PWM 27  // سرعة الموتور اليمين (M2)
#define PIN_LATCH 18   // خط القفل
#define PIN_CLK 19     // خط النبضات
#define PIN_DATA 23    // خط البيانات
#define PIN_EN 22      // خط تفعيل الشريحة (Active Low)

// --- PWM SETTINGS ---
const int pwm_freq = 5000;
const int pwm_res = 8;

// --- ROS ENTITIES ---
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_publisher_t pub_line_error;
rcl_publisher_t pub_enc_l, pub_enc_r;

std_msgs__msg__Float32 msg_line_error;
std_msgs__msg__Int32 msg_enc_l, msg_enc_r;

// --- STATE ---
volatile long enc_l_ticks = 0;
volatile long enc_r_ticks = 0;
float prev_error = 0.0;
float integral = 0.0;
float kp = 1.0;
float ki = 0.0;
float kd = 0.1;
float base_speed = 0.3;

unsigned long last_pub_time = 0;

// متغير لحفظ حالة اتجاه المواتير في الـ Shift Register
uint8_t latch_state = 0; 

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
    latch_state |= bit_A;   // أمامي
    latch_state &= ~bit_B;
    ledcWrite(pin_pwm, pwm);
  } else if (pwm < 0) {
    latch_state &= ~bit_A;  // خلفي
    latch_state |= bit_B;
    ledcWrite(pin_pwm, -pwm);
  } else {
    latch_state &= ~bit_A;  // توقف
    latch_state &= ~bit_B;
    ledcWrite(pin_pwm, 0);
  }
  
  // تطبيق الاتجاه فوراً
  updateShiftRegister();
}

void setup() {
  pinMode(PIN_LED, OUTPUT);

  // إعداد الواي فاي والـ Agent
  char ssid[] = "wafaa hammad";              
  char psk[] = "12345678";   
  char agent_ip[] = "192.168.137.88"; 
  size_t agent_port = 8888;           
  
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  WiFi.setSleep(false); // إلغاء خمول الواي فاي لضمان استقرار الاتصال

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

  // Encoders 
  pinMode(PIN_ENC_L, INPUT_PULLUP);
  pinMode(PIN_ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L), count_l, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R), count_r, RISING);

  delay(2000);
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_bridge", "", &support));

  // Publishers
  RCCHECK(rclc_publisher_init_default(&pub_line_error, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "line_error"));
  RCCHECK(rclc_publisher_init_default(&pub_enc_l, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "raw/encoder_l"));
  RCCHECK(rclc_publisher_init_default(&pub_enc_r, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "raw/encoder_r"));

  digitalWrite(PIN_LED, HIGH);  
}

void loop() {
  // حساب خطأ المسار محلياً على ESP32
  int ir_pins[] = { PIN_IR_L2, PIN_IR_L1, PIN_IR_MID, PIN_IR_R1, PIN_IR_R2 };
  float weights[] = { 2.0, 1.0, 0.0, -1.0, -2.0 };
  int threshold = 2000;
  
  float total_weight = 0.0;
  float total_on_line = 0.0;
  
  for (int i = 0; i < 5; i++) {
    int raw_val = analogRead(ir_pins[i]);
    if (raw_val < threshold) {
      total_weight += weights[i];
      total_on_line += 1.0;
    }
  }
  
  if (total_on_line > 0) {
    msg_line_error.data = total_weight / total_on_line;
  } else {
    msg_line_error.data = 0.0;
  }
  RCSOFTCHECK(rcl_publish(&pub_line_error, &msg_line_error, NULL));

  // حماية قراءة الـ Encoders
  noInterrupts();
  long current_enc_l = enc_l_ticks;
  long current_enc_r = enc_r_ticks;
  interrupts();

  msg_enc_l.data = current_enc_l;
  msg_enc_r.data = current_enc_r;
  RCSOFTCHECK(rcl_publish(&pub_enc_l, &msg_enc_l, NULL));
  RCSOFTCHECK(rcl_publish(&pub_enc_r, &msg_enc_r, NULL));

  // Run PID controller locally
  float error = msg_line_error.data;
  integral += error;
  float derivative = error - prev_error;
  float angular_z = (kp * error) + (ki * integral) + (kd * derivative);
  prev_error = error;

  float v = base_speed;
  float wheel_separation = 0.135;

  float l_raw = v - (angular_z * wheel_separation / 2.0);
  float r_raw = v + (angular_z * wheel_separation / 2.0);

  int pwm_l = (int)(l_raw * 255.0);
  int pwm_r = (int)(r_raw * 255.0);

  if (pwm_l > 255) pwm_l = 255;
  if (pwm_l < -255) pwm_l = -255;
  if (pwm_r > 255) pwm_r = 255;
  if (pwm_r < -255) pwm_r = -255;

  // إرسال السرعات للمحركات باستخدام الدالة الجديدة
  // 1 = الموتور اليسار، 2 = الموتور اليمين
  set_motor_speed(1, pwm_l);
  set_motor_speed(2, pwm_r);

  delay(10);
}