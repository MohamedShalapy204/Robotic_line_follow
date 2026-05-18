#include <Arduino.h>

// --- PIN DEFINITIONS ---
#define PIN_IR_L2 36
#define PIN_IR_L1 39
#define PIN_IR_MID 34
#define PIN_IR_R1 35
#define PIN_IR_R2 32

// --- L293D Shift Register Pins ---
#define PIN_M1_PWM 14  // Left Motor (M1) Speed
#define PIN_M2_PWM 27  // Right Motor (M2) Speed
#define PIN_LATCH 18   // Latch Pin
#define PIN_CLK 19     // Clock Pin
#define PIN_DATA 23    // Data Pin
#define PIN_EN 22      // Enable Pin (Active Low)

#define PIN_LED 13

const int ir_pins[] = { PIN_IR_L2, PIN_IR_L1, PIN_IR_MID, PIN_IR_R1, PIN_IR_R2 };

// --- PWM SETTINGS ---
const int pwm_freq = 5000;
const int pwm_res = 8;

// --- ROBOT SETTINGS ---
int hw_threshold = 2000;  // Threshold for detecting the line
float kp = 1.2;
float kd = 0.1;
float base_speed = 150.0; // Base PWM speed (0-255)
float prev_error = 0.0;

// Trim for the stronger left wheel
float left_trim_factor = 1.145; 

// State Machine
enum State { FOLLOWING, STOPPED, ROTATING, FINISHED };
State robotState = FOLLOWING;
unsigned long stop_time = 0;

uint8_t latch_state = 0;

// متغيرات لحساب عدد اللفات في المسار
int lap_counter = 0;           // العداد
bool on_finish_line = false;   // لمنع العداد من الزيادة المستمرة أثناء الوقوف على الخط

// Update the Shift Register for motor directions
void updateShiftRegister() {
  digitalWrite(PIN_LATCH, LOW);
  shiftOut(PIN_DATA, PIN_CLK, MSBFIRST, latch_state);
  digitalWrite(PIN_LATCH, HIGH);
}

// Set motor speed and direction (1 for Left, 2 for Right)
void set_motor_speed(int motor, int pwm) {
  int pin_pwm;
  uint8_t bit_A, bit_B;

  // Trim the left wheel to compensate for it being stronger
  if (motor == 1) {
    pwm = (int)(pwm / left_trim_factor);
  }

  // Cap PWM to allowable limits
  if (pwm > 255) pwm = 255;
  if (pwm < -255) pwm = -255;

  if (motor == 1) { 
    pin_pwm = PIN_M1_PWM;
    bit_A = 4; // Bit 2
    bit_B = 8; // Bit 3
  } else {          
    pin_pwm = PIN_M2_PWM;
    bit_A = 2;  // Bit 1
    bit_B = 16; // Bit 4
  }

  if (pwm > 0) {
    latch_state &= ~bit_A;  // Forward
    latch_state |= bit_B;
    ledcWrite(pin_pwm, pwm);
  } else if (pwm < 0) {
    latch_state |= bit_A;   // Backward
    latch_state &= ~bit_B;
    ledcWrite(pin_pwm, -pwm);
  } else {
    latch_state &= ~bit_A;  // Stop
    latch_state &= ~bit_B;
    ledcWrite(pin_pwm, 0);
  }
  
  updateShiftRegister();
}

void setup() {
  pinMode(PIN_LED, OUTPUT);

  // Setup IR Sensors
  for (int i = 0; i < 5; i++) {
    pinMode(ir_pins[i], INPUT);
  }

  // Setup Motor Pins
  pinMode(PIN_LATCH, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  
  digitalWrite(PIN_EN, LOW); // Enable L293D
  updateShiftRegister();     // Stop motors initially

  ledcAttach(PIN_M1_PWM, pwm_freq, pwm_res);
  ledcAttach(PIN_M2_PWM, pwm_freq, pwm_res);

  // الليد يكون مطفأ في البداية عند تشغيل الروبوت
  digitalWrite(PIN_LED, LOW); 
  delay(2000); // Give time before moving
}

void loop() {
  // Read Sensors
  float sensor_values[5];
  float total_on_line = 0;
  
  for (int i = 0; i < 5; i++) {
    int raw_val = analogRead(ir_pins[i]);
    float is_line = (raw_val < hw_threshold) ? 1.0 : 0.0;
    sensor_values[i] = is_line;
    total_on_line += is_line;
  }

  // --- STATE MACHINE ---
  switch (robotState) {
    
    case FOLLOWING:
      // اكتشاف خط البداية/النهاية العرضي (قراءة 4 حساسات أو أكثر للون الأسود)
      // اكتشاف خط البداية/النهاية العرضي (قراءة 4 حساسات أو أكثر للون الأسود)
      if (total_on_line >= 4) {
        if (on_finish_line == false) {
          lap_counter++;               // زود عداد اللفات
          on_finish_line = true;       // تأمين العداد لتسجيل لفة واحدة فقط في كل مرور
          if (lap_counter!=1){
            digitalWrite(PIN_LED, HIGH); // <-- التعديل هنا: نور الليد أول ما يلمس الخط
          }
        }
      } else {
        on_finish_line = false;        // إعادة فتح العداد عند مغادرة الخط العرضي
        digitalWrite(PIN_LED, LOW);    // <-- التعديل هنا: اطفي الليد أول ما يسيب الخط عشان تجهز للفة الجاية
      }

      // هل أتم الروبوت لفتين كاملتين؟ (بافتراض إنه بدأ قبل الخط)
      if (lap_counter >= 3) {
        set_motor_speed(1, 0);         // إيقاف المحرك الأيسر
        set_motor_speed(2, 0);         // إيقاف المحرك الأيمن
        digitalWrite(PIN_LED, HIGH);   // <-- تأكيد إن الليد تفضل منورة في النهاية وماتطفيش
        robotState = FINISHED;         // الانتقال لحالة النهاية التامة
      } 
      // إذا فقد الخط تماماً
      else if (total_on_line == 0) {      // الانتقال لحالة النهاية التامة
      } 
      // إذا فقد الخط تماماً
      else if (total_on_line == 0) {
        set_motor_speed(1, 0);
        set_motor_speed(2, 0);
        stop_time = millis();
        robotState = STOPPED;
      } 
      // التتبع العادي للمسار باستخدام الـ PID
      else {
        // Calculate Error
        float weights[] = { 2.0, 1.0, 0.0, -1.0, -2.0 };
        float sum_weighted = 0;
        for (int i = 0; i < 5; i++) {
          sum_weighted += sensor_values[i] * weights[i];
        }
        float error = sum_weighted / total_on_line;

        // PID Calculation
        float derivative = error - prev_error;
        float correction = (kp * error * 50) + (kd * derivative * 50); 
        prev_error = error;

        // Calculate Motor Speeds
        int pwm_l = base_speed + correction;
        int pwm_r = base_speed - correction;

        // Apply Deadband (Minimum PWM to overcome friction)
        int min_pwm = 90;
        if (pwm_l > 0 && pwm_l < min_pwm) pwm_l = min_pwm;
        if (pwm_r > 0 && pwm_r < min_pwm) pwm_r = min_pwm;

        set_motor_speed(1, pwm_l);
        set_motor_speed(2, pwm_r);
      }
      break;

    case STOPPED:
      // Wait for 1 second before rotating
      if (millis() - stop_time > 1000) {
        robotState = ROTATING;
      }
      break;

    case ROTATING:
      // Spin in place (Left backward, Right forward)
      set_motor_speed(1, -120);
      set_motor_speed(2, 120);

      // If the middle sensor or any inner sensor sees the line, resume following
      if (sensor_values[2] > 0 || sensor_values[1] > 0 || sensor_values[3] > 0) {
        set_motor_speed(1, 0);
        set_motor_speed(2, 0);
        delay(200); // Brief pause to stabilize
        prev_error = 0.0;
        robotState = FOLLOWING;
      }
      break;

    case FINISHED:
      // الروبوت يبقى متوقفاً والليد يظل مضيئاً حتى يتم إعادة التشغيل
      set_motor_speed(1, 0);
      set_motor_speed(2, 0);
      break;
  }

  delay(10); // Small loop delay for stability
}