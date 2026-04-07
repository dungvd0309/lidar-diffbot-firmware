#include <Arduino.h>
#include "config.h"
#include "motor_controller.h"

CONFIG cfg;

MotorController leftMotor, rightMotor;
void set_motor_pwm(MotorController * motor, float pwm);

// Handle interrupts
void IRAM_ATTR quad_encoder_a_left_isr() {
  byte enc_a = digitalRead(CONFIG::mot_left_enc_gpio_a_fg);
  byte enc_b = digitalRead(CONFIG::mot_left_enc_gpio_b);
  leftMotor.tickEncoder(enc_a == enc_b);
}

void IRAM_ATTR quad_encoder_a_right_isr() {
  byte enc_a = digitalRead(CONFIG::mot_right_enc_gpio_a_fg);
  byte enc_b = digitalRead(CONFIG::mot_right_enc_gpio_b);
  rightMotor.tickEncoder(enc_a == enc_b);
}

void IRAM_ATTR quad_encoder_b_left_isr() {
  byte enc_a = digitalRead(CONFIG::mot_left_enc_gpio_a_fg);
  byte enc_b = digitalRead(CONFIG::mot_left_enc_gpio_b);
  leftMotor.tickEncoder(enc_a != enc_b);
}

void IRAM_ATTR quad_encoder_b_right_isr() {
  byte enc_a = digitalRead(CONFIG::mot_right_enc_gpio_a_fg);
  byte enc_b = digitalRead(CONFIG::mot_right_enc_gpio_b);
  rightMotor.tickEncoder(enc_a != enc_b);
}

void encoders_init() {
  pinMode(CONFIG::mot_left_enc_gpio_a_fg, INPUT_PULLUP);
  pinMode(CONFIG::mot_left_enc_gpio_b, INPUT_PULLUP);
  attachInterrupt(CONFIG::mot_left_enc_gpio_a_fg, quad_encoder_a_left_isr, CHANGE);
  attachInterrupt(CONFIG::mot_left_enc_gpio_b, quad_encoder_b_left_isr, CHANGE);

  pinMode(CONFIG::mot_right_enc_gpio_a_fg, INPUT_PULLUP);
  pinMode(CONFIG::mot_right_enc_gpio_b, INPUT_PULLUP);
  attachInterrupt(CONFIG::mot_right_enc_gpio_a_fg, quad_encoder_a_right_isr, CHANGE);
  attachInterrupt(CONFIG::mot_right_enc_gpio_b, quad_encoder_b_right_isr, CHANGE);
}

void motors_init() {
  // in 1,2,3,4 pins setup
  pinMode(CONFIG::mot_left_drv_gpio_in1, OUTPUT);
  pinMode(CONFIG::mot_left_drv_gpio_in2, OUTPUT);
  pinMode(CONFIG::mot_right_drv_gpio_in1, OUTPUT);
  pinMode(CONFIG::mot_right_drv_gpio_in2, OUTPUT);

  // PWM pins setup
  pinMode(CONFIG::mot_left_drv_gpio_pwm, OUTPUT);
  pinMode(CONFIG::mot_right_drv_gpio_pwm, OUTPUT);
  ledcSetup(CONFIG::MOT_PWM_LEFT_CHANNEL, CONFIG::PWM_FREQUENCY, CONFIG::PWM_RESOLUTION);
  ledcSetup(CONFIG::MOT_PWM_RIGHT_CHANNEL, CONFIG::PWM_FREQUENCY, CONFIG::PWM_RESOLUTION);

  leftMotor.init(CONFIG::WHEEL_CPR, CONFIG::motor_driver_pid_kp, CONFIG::motor_driver_pid_ki, CONFIG::motor_driver_pid_kd, CONFIG::motor_max_rpm, CONFIG::motor_left_direction, CONFIG::encoder_left_direction);
  rightMotor.init(CONFIG::WHEEL_CPR, CONFIG::motor_driver_pid_kp, CONFIG::motor_driver_pid_ki, CONFIG::motor_driver_pid_kd, CONFIG::motor_max_rpm, CONFIG::motor_right_direction, CONFIG::encoder_right_direction);

  leftMotor.setPWMCallback(set_motor_pwm);
  rightMotor.setPWMCallback(set_motor_pwm);

  leftMotor.enablePID(true);
  rightMotor.enablePID(true);
}

void set_motor_pwm(MotorController * motor, float pwm) {
  bool is_right = motor == &rightMotor;

  uint8_t pwm_channel = is_right ? cfg.MOT_PWM_RIGHT_CHANNEL : cfg.MOT_PWM_LEFT_CHANNEL;
  uint8_t pwm_pin = is_right ? cfg.mot_right_drv_gpio_pwm : cfg.mot_left_drv_gpio_pwm;
  uint8_t in1_pin = is_right ? cfg.mot_right_drv_gpio_in1 : cfg.mot_left_drv_gpio_in1;
  uint8_t in2_pin = is_right ? cfg.mot_right_drv_gpio_in2 : cfg.mot_left_drv_gpio_in2;

  int max_pwm = cfg.motor_driver_max_pwm;
  int min_pwm = cfg.motor_driver_min_pwm;

  if (pwm == 0) {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, HIGH);
  }
  else if (pwm > 0) {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
  }
  else {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
  }

  int pwm_magnitude = round(abs(pwm) * max_pwm);
  int mapped_pwm = map(pwm_magnitude, 0, max_pwm, min_pwm, max_pwm);
  ledcAttachPin(pwm_pin, pwm_channel);
  ledcWrite(pwm_channel, mapped_pwm);
}

void set_motors_rpm(float left_rpm, float right_rpm) {
  leftMotor.setTargetRPM(left_rpm);
  rightMotor.setTargetRPM(right_rpm);
}

void get_motors_rpm(float *left_rpm, float *right_rpm) {
  *left_rpm = leftMotor.getCurrentRPM();
  *right_rpm = rightMotor.getCurrentRPM();
}

void get_motors_pos(float *left_pos, float *right_pos) {
  *left_pos = leftMotor.getEncoderRadValue();
  *right_pos = rightMotor.getEncoderRadValue();
}

void motors_update() {
  leftMotor.update();
  rightMotor.update();
}