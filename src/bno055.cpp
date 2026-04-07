#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

#include "config.h"
#include "bno055.h"

static Adafruit_BNO055 bno = Adafruit_BNO055(55, CONFIG::bno055_addr, &Wire);
static bool g_imu_ready = false;

bool imu_init() {
    Wire.begin(CONFIG::bno055_sda, CONFIG::bno055_scl);

    if (!bno.begin()) {
        Serial.println("Failed to initialize BNO055. Check wiring/address.");
        g_imu_ready = false;
        return false;
    }

    delay(50);
    bno.setExtCrystalUse(true);
    g_imu_ready = true;
    return true;
}

bool imu_read(ImuData *out_data) {
    if (out_data == NULL || !g_imu_ready) {
        return false;
    }

    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> linear_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    out_data->orientation_w = quat.w();
    out_data->orientation_x = quat.x();
    out_data->orientation_y = quat.y();
    out_data->orientation_z = quat.z();

    out_data->angular_velocity_x = gyro.x();
    out_data->angular_velocity_y = gyro.y();
    out_data->angular_velocity_z = gyro.z();

    out_data->linear_acceleration_x = linear_accel.x();
    out_data->linear_acceleration_y = linear_accel.y();
    out_data->linear_acceleration_z = linear_accel.z();

    uint8_t sys = 0;
    uint8_t gyro_cal = 0;
    uint8_t accel_cal = 0;
    uint8_t mag = 0;
    bno.getCalibration(&sys, &gyro_cal, &accel_cal, &mag);
    out_data->is_calibrated = (sys >= 1 && gyro_cal >= 1 && accel_cal >= 1);

    return true;
}