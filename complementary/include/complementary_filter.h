#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

struct complementary_filter {
	float angle_deg;
	float alpha;
};

/*
 * Start the filter at an angle in degrees. Finite alpha values are clamped to
 * [0, 1]; a non-finite alpha uses 0.98. A non-finite angle starts at 0.
 */
void complementary_filter_init(struct complementary_filter *filter,
			       float initial_angle_deg, float alpha);

/*
 * Set a finite angle in degrees without changing alpha. Invalid input is
 * ignored.
 */
void complementary_filter_reset(struct complementary_filter *filter,
				float angle_deg);

/*
 * Update with degrees/second, degrees, and seconds. Invalid input leaves the
 * last finite angle unchanged. Accelerometer correction takes the shortest
 * path across the +/-180-degree boundary; gyro integration is not wrapped.
 */
float complementary_filter_update(struct complementary_filter *filter,
				  float gyro_rate_dps,
				  float accel_angle_deg, float dt_s);

#ifdef __cplusplus
}
#endif

#endif
