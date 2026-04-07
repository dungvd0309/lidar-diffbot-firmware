#pragma once

struct ImuData {
	float orientation_w;
	float orientation_x;
	float orientation_y;
	float orientation_z;

	float angular_velocity_x;
	float angular_velocity_y;
	float angular_velocity_z;

	float linear_acceleration_x;
	float linear_acceleration_y;
	float linear_acceleration_z;

	bool is_calibrated;
};

bool imu_init();
bool imu_read(ImuData *out_data);