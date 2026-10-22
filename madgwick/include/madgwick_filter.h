/*
 * madgwick_filter.h - orientation from gyro + accel (+ mag) as a quaternion.
 *
 * Frames, stated once and used everywhere in this library:
 *
 *   Sensor frame  x forward, y left, z up.
 *   Earth frame   x north (magnetic), y west, z up. Both are right-handed.
 *   Quaternion    q = (w, x, y, z), the rotation from the earth frame to the
 *                 sensor frame, always unit length.
 *   Gravity       a level, still sensor reads +1 g on its z axis, the way a
 *                 real accelerometer reports the reaction force holding it up.
 *   Angles        roll about x, pitch about y, yaw about z, right-hand
 *                 positive. Yaw 0 means the sensor x axis points north.
 */
#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

struct madgwick_filter {
	float q[4];	/* w, x, y, z: rotation from the earth frame to the
			 * sensor frame, unit length */
	float beta;	/* correction gain. Larger trusts the accelerometer/
			 * magnetometer more and settles faster but jitters
			 * more; 0.1 is a good start */
};

/* Start level, facing the earth-frame x axis. A non-finite or negative beta
 * uses 0.1. */
void madgwick_filter_init(struct madgwick_filter *filter, float beta);

/* Put the filter at a known orientation. NULL q means level. Invalid input is
 * ignored. */
void madgwick_filter_reset(struct madgwick_filter *filter, const float q[4]);

/* Gyro in rad/s, accelerometer in any consistent unit (only its direction is
 * used), dt in seconds. Non-finite input or dt <= 0 leaves the state
 * unchanged. A zero accelerometer vector skips the correction for that step
 * (free fall). */
void madgwick_filter_update_imu(struct madgwick_filter *filter,
				float gx, float gy, float gz,
				float ax, float ay, float az, float dt_s);

/* Same, with a magnetometer in any consistent unit. A zero magnetometer vector
 * falls back to madgwick_filter_update_imu. */
void madgwick_filter_update(struct madgwick_filter *filter,
			    float gx, float gy, float gz,
			    float ax, float ay, float az,
			    float mx, float my, float mz, float dt_s);

/* Roll, pitch, yaw in degrees (x-forward, y-left, z-up; yaw about z). Any
 * pointer may be NULL. */
void madgwick_filter_euler(const struct madgwick_filter *filter,
			   float *roll_deg, float *pitch_deg, float *yaw_deg);

#ifdef __cplusplus
}
#endif

#endif
