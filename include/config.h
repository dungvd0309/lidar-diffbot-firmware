#pragma once

#include <cstdint>

class CONFIG
{
public:
    //=== Motor and encoder config ===//
    static constexpr float WHEEL_DIAMETER = 0.065;             // m
    static constexpr float WHEEL_RADIUS = WHEEL_DIAMETER * 0.5;  // m
    static constexpr float GEAR_RATIO = 20.4068;
    static constexpr int ENCODER_CPR = 11;  // số xung trên 1 vòng encoder (COUNTS_PER_REVOLUTION)
    static constexpr int DECODE_FACTOR = 4; // hệ số giải mã xung A/B (x1, x2, x4)
    static constexpr float WHEEL_CPR = GEAR_RATIO * ENCODER_CPR * DECODE_FACTOR; // số xung trên 1 vòng trục (COUNTS_PER_REVOLUTION)

    static constexpr int PWM_FREQUENCY = 20000; // Hz
    static constexpr uint8_t PWM_RESOLUTION = 8; // Bits

    static constexpr float base_wheel_track = 0.18f; // khoang cach giua 2 banh xe (m)
    static constexpr float wheel_perim_len_div60 = PI * WHEEL_DIAMETER / 60;
    static constexpr float wheel_perim_len_div60_recip = 1/wheel_perim_len_div60;

    // Motor driver config
    static constexpr float motor_max_rpm = 200;
    static constexpr int motor_driver_max_pwm = 255;
    static constexpr int motor_driver_min_pwm = 135;
    static constexpr float motor_driver_pid_kp = 0.002f;
    static constexpr float motor_driver_pid_ki = 0.0015f;
    static constexpr float motor_driver_pid_kd = 0;
    // Use 1 for normal direction, -1 for inverted direction.
    static constexpr int8_t motor_left_direction = 1;
    static constexpr int8_t encoder_left_direction = -1;
    static constexpr int8_t motor_right_direction = -1;
    static constexpr int8_t encoder_right_direction = 1;

    // motor driver pins
    static constexpr uint8_t mot_left_drv_gpio_pwm = 14; // ENA
    static constexpr uint8_t mot_left_drv_gpio_in1 = 27; // IN1
    static constexpr uint8_t mot_left_drv_gpio_in2 = 26; // IN2
    static constexpr uint8_t mot_right_drv_gpio_in1 = 25;// IN3
    static constexpr uint8_t mot_right_drv_gpio_in2 = 33;// IN4
    static constexpr uint8_t mot_right_drv_gpio_pwm = 32;// ENB

    // encoder pins
    static constexpr uint8_t mot_left_enc_gpio_a_fg = 16;
    static constexpr uint8_t mot_left_enc_gpio_b = 17;
    static constexpr uint8_t mot_right_enc_gpio_a_fg = 18;
    static constexpr uint8_t mot_right_enc_gpio_b = 19;

    static constexpr uint8_t MOT_PWM_LEFT_CHANNEL = 0;
    static constexpr uint8_t MOT_PWM_RIGHT_CHANNEL = 1;
    //=== === === === ===//

    //=== Kinematics config ===//
    static constexpr float speed_to_rpm(float speed_ms) {
        return speed_ms*wheel_perim_len_div60_recip;
    }

    static constexpr float rpm_to_speed(float rpm) {
        return rpm*wheel_perim_len_div60;
    }

    static void twistToWheelSpeeds(float speed_lin_x, float speed_ang_z,
                                    float *speed_right, float *speed_left) {
        float ang_component = speed_ang_z*base_wheel_track*0.5f;
        *speed_right = speed_lin_x + ang_component;
        *speed_left  = speed_lin_x - ang_component;
    }
    //=== === === === ===//

    //=== Battery monitoring config === //
    static constexpr uint8_t battery_voltage_pin = 34;
    static constexpr uint8_t adc_resolution = 12;
    static constexpr float adc_max_value = (1 << CONFIG::adc_resolution) - 1;

    static constexpr float battery_r1_ohm = 9470.0f; // 10k ohm res (5%)
    static constexpr float battery_r2_ohm = 3300.0f; // 3.3k ohm res (5%)
    static constexpr float battery_voltage_divider_ratio = battery_r2_ohm / (battery_r1_ohm + battery_r2_ohm);

    static constexpr float battery_max_voltage = 12.2;   // 3S LiIon
    static constexpr float battery_min_voltage = 9.2;    // 3S LiIon
    static constexpr float battery_adc_reference_voltage = 3.3;      // ESP32 ADC reference voltage
    static constexpr float battery_ema_alpha = 0.05f;    // EMA filter alpha
    //=== === === === ===//

    //=== BNO055 IMU config ===//
    static constexpr uint8_t bno055_addr = 0x29;
    static constexpr uint8_t bno055_sda = 21;
    static constexpr uint8_t bno055_scl = 22;
    static constexpr float imu_orientation_covariance = 0.0025f;  // rough estimate
    static constexpr float imu_angular_velocity_covariance = 0.0001f;
    static constexpr float imu_linear_acceleration_covariance = 0.0005f;
    //=== === === === ===//

};
