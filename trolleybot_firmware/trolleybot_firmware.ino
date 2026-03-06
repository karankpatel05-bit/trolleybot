// TrolleyBot Arduino Nano Firmware
// Handles N20 Motors with Encoders and Serial Communication with RPi

// --- Pin Definitions ---
// Motor 1 (Left Side)
const int LEFT_ENCA = 2; // Yellow wire -> D2 (Interrupt 0)
const int LEFT_ENCB = 4; // Green wire -> D4 (Direction)
const int LEFT_PWM = 10; // ENA -> D10
const int LEFT_IN1 = 9;  // IN1 -> D9
const int LEFT_IN2 = 8;  // IN2 -> D8

// Motor 2 (Right Side)
const int RIGHT_ENCA = 3; // Yellow wire -> D3 (Interrupt 1)
const int RIGHT_ENCB = 11; // Green wire -> D11 (Direction)
const int RIGHT_PWM = 5;  // ENB -> D5
const int RIGHT_IN1 = 7;  // IN3 -> D7
const int RIGHT_IN2 = 6;  // IN4 -> D6

// --- Variables ---
volatile long left_ticks = 0;
volatile long right_ticks = 0;

long prev_left_ticks = 0;
long prev_right_ticks = 0;

// PID Variables
float kp = 1.0, ki = 0.0, kd = 0.1;

float left_target_vel = 0; // target ticks per frame
float right_target_vel = 0;

float left_integral = 0;
float right_integral = 0;

long prev_time = 0;
const int LOOP_TIME = 50; // ms (20Hz)

void setup() {
  Serial.begin(115200);

  // Motor Pins
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  // Encoder Pins
  pinMode(LEFT_ENCA, INPUT_PULLUP);
  pinMode(LEFT_ENCB, INPUT_PULLUP);
  pinMode(RIGHT_ENCA, INPUT_PULLUP);
  pinMode(RIGHT_ENCB, INPUT_PULLUP);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCA), left_encoder_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCA), right_encoder_isr, RISING);
}

void loop() {
  long current_time = millis();
  
  // Read Serial Commands (format: "L,R\n" target velocities)
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    int comma_idx = cmd.indexOf(',');
    if (comma_idx != -1) {
      left_target_vel = cmd.substring(0, comma_idx).toFloat();
      right_target_vel = cmd.substring(comma_idx + 1).toFloat();
    }
  }

  if (current_time - prev_time >= LOOP_TIME) {
    // 1. Calculate current velocity (ticks per frame)
    long current_left_ticks = left_ticks;
    long current_right_ticks = right_ticks;
    
    float left_current_vel = current_left_ticks - prev_left_ticks;
    float right_current_vel = current_right_ticks - prev_right_ticks;
    
    prev_left_ticks = current_left_ticks;
    prev_right_ticks = current_right_ticks;

    // 2. PID Control
    int left_pwm = calculate_pid(left_target_vel, left_current_vel, &left_integral);
    int right_pwm = calculate_pid(right_target_vel, right_current_vel, &right_integral);

    // 3. Set Motor Speeds
    set_motor(1, left_pwm);
    set_motor(2, right_pwm);

    // 4. Send Odometry over Serial (format: "L_TICKS,R_TICKS\n")
    Serial.print(current_left_ticks);
    Serial.print(",");
    Serial.println(current_right_ticks);

    prev_time = current_time;
  }
}

// --- PID Implementation ---
int calculate_pid(float target, float current, float* integral) {
  if (target == 0) {
    *integral = 0;
    return 0;
  }
  
  float error = target - current;
  *integral += error;
  
  // Anti-windup
  if (*integral > 255) *integral = 255;
  if (*integral < -255) *integral = -255;
  
  float derivative = error; // Simplified for fixed time step
  
  float output = (kp * error) + (ki * *integral) + (kd * derivative);
  
  return (int)constrain(output, -255, 255);
}

// --- Motor Control ---
void set_motor(int motor, int pwm) {
  int dir = (pwm >= 0) ? 1 : 0;
  pwm = abs(pwm);
  if (pwm > 255) pwm = 255;

  if (motor == 1) { // Left
    analogWrite(LEFT_PWM, pwm);
    if (dir == 1) {
      digitalWrite(LEFT_IN1, HIGH);
      digitalWrite(LEFT_IN2, LOW);
    } else {
      digitalWrite(LEFT_IN1, LOW);
      digitalWrite(LEFT_IN2, HIGH);
    }
    if (pwm == 0) {
      digitalWrite(LEFT_IN1, LOW);
      digitalWrite(LEFT_IN2, LOW);
    }
  } else if (motor == 2) { // Right
    analogWrite(RIGHT_PWM, pwm);
    if (dir == 1) {
      digitalWrite(RIGHT_IN1, HIGH);
      digitalWrite(RIGHT_IN2, LOW);
    } else {
      digitalWrite(RIGHT_IN1, LOW);
      digitalWrite(RIGHT_IN2, HIGH);
    }
    if (pwm == 0) {
      digitalWrite(RIGHT_IN1, LOW);
      digitalWrite(RIGHT_IN2, LOW);
    }
  }
}

// --- Interrupt Service Routines ---
void left_encoder_isr() {
  if (digitalRead(LEFT_ENCB) == LOW) {
    left_ticks++;
  } else {
    left_ticks--;
  }
}

void right_encoder_isr() {
  if (digitalRead(RIGHT_ENCB) == LOW) {
    right_ticks++;
  } else {
    right_ticks--;
  }
}
