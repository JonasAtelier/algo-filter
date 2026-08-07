#include <complementary_filter.h>
#include <stdint.h>

struct complementary_filter angle_filter;
uint32_t previous_time_us;

static const float MAX_DT_S = 0.1f;

void setup()
{
	/*
	 * Replace this placeholder with the first accelerometer angle sample.
	 * Starting there avoids a large correction from an arbitrary zero angle.
	 */
	float initial_accel_angle_deg = 0.0f;

	Serial.begin(115200);
	complementary_filter_init(&angle_filter, initial_accel_angle_deg, 0.98f);
	previous_time_us = micros();
}

void loop()
{
	uint32_t current_time_us = micros();
	uint32_t elapsed_time_us = current_time_us - previous_time_us;
	float dt_s = (float)elapsed_time_us / 1000000.0f;

	previous_time_us = current_time_us;

	/*
	 * Replace these placeholders with readings for the same rotation axis.
	 * Positive gyro motion and positive accelerometer tilt must agree.
	 */
	float gyro_rate_dps = 0.0f;
	float accel_angle_deg = 0.0f;

	/*
	 * Unsigned subtraction handles one micros() counter wrap. Reject a long
	 * stall and resynchronize instead of integrating it as real motion.
	 */
	if (dt_s <= 0.0f || dt_s > MAX_DT_S) {
		delay(10);
		return;
	}

	float angle_deg = complementary_filter_update(&angle_filter,
						      gyro_rate_dps,
						      accel_angle_deg,
						      dt_s);

	Serial.println(angle_deg);
	delay(10);
}
