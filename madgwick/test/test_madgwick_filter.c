#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "madgwick_filter.h"

#define FLOAT_TOLERANCE	0.0001f
#define ANGLE_TOLERANCE	0.5f		/* degrees */
#define GRAVITY		9.81f		/* m/s^2, any unit would do */
#define DEG_TO_RAD	0.017453292519943295f
#define RAD_TO_DEG	57.29577951308232f

static int tests_run;

static void expect_within(float actual, float expected, float tolerance,
			  const char *name)
{
	tests_run++;
	if (isfinite(actual) && fabsf(actual - expected) <= tolerance)
		return;

	fprintf(stderr, "FAIL: %s: expected %.6f +/- %.6f, got %.6f\n",
		name, expected, tolerance, actual);
	exit(EXIT_FAILURE);
}

static void expect_close(float actual, float expected, const char *name)
{
	expect_within(actual, expected, FLOAT_TOLERANCE, name);
}

static void expect_quaternion(const struct madgwick_filter *filter,
			      float w, float x, float y, float z,
			      const char *name)
{
	expect_close(filter->q[0], w, name);
	expect_close(filter->q[1], x, name);
	expect_close(filter->q[2], y, name);
	expect_close(filter->q[3], z, name);
}

/*
 * Earth-frame gravity (0, 0, GRAVITY) as a sensor rolled by roll_deg about its
 * x axis reads it. The quaternion for that roll is (cos(r/2), sin(r/2), 0, 0),
 * and rotating the earth z axis into the sensor frame with it leaves
 * (0, 2*q0*q1, 1 - 2*q1*q1) -- the same convention madgwick_filter_euler
 * reports back.
 */
static void gravity_at_roll(float roll_deg, float accel[3])
{
	float half_rad = roll_deg * DEG_TO_RAD * 0.5f;
	float q0 = cosf(half_rad);
	float q1 = sinf(half_rad);

	accel[0] = 0.0f;
	accel[1] = GRAVITY * 2.0f * q0 * q1;
	accel[2] = GRAVITY * (1.0f - 2.0f * q1 * q1);
}

/* Deterministic pseudo-random floats in [-1, 1], as examples/tilt.c uses. */
static float jitter(unsigned long *seed)
{
	*seed = *seed * 1103515245UL + 12345UL;

	return (float)((*seed >> 16) % 2000UL) / 1000.0f - 1.0f;
}

static void test_null_handling(void)
{
	float roll_deg = 42.0f;

	madgwick_filter_init(NULL, 0.1f);
	madgwick_filter_reset(NULL, NULL);
	madgwick_filter_update_imu(NULL, 0.0f, 0.0f, 0.0f,
				   0.0f, 0.0f, GRAVITY, 0.01f);
	madgwick_filter_update(NULL, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, GRAVITY,
			       1.0f, 0.0f, 0.0f, 0.01f);
	madgwick_filter_euler(NULL, &roll_deg, NULL, NULL);
	expect_close(roll_deg, 42.0f, "null euler leaves output alone");
}

static void test_init(void)
{
	struct madgwick_filter filter;

	madgwick_filter_init(&filter, 0.25f);
	expect_quaternion(&filter, 1.0f, 0.0f, 0.0f, 0.0f, "init identity");
	expect_close(filter.beta, 0.25f, "init keeps a valid beta");

	madgwick_filter_init(&filter, 0.0f);
	expect_close(filter.beta, 0.0f, "init keeps zero beta");

	madgwick_filter_init(&filter, -1.0f);
	expect_close(filter.beta, 0.1f, "negative beta defaults");

	madgwick_filter_init(&filter, NAN);
	expect_close(filter.beta, 0.1f, "NaN beta defaults");

	madgwick_filter_init(&filter, INFINITY);
	expect_close(filter.beta, 0.1f, "infinite beta defaults");
}

static void test_reset(void)
{
	struct madgwick_filter filter;
	const float yaw_90[4] = { 0.70710678f, 0.0f, 0.0f, 0.70710678f };
	const float unnormalised[4] = { 2.0f, 0.0f, 0.0f, 0.0f };
	const float not_finite[4] = { 1.0f, NAN, 0.0f, 0.0f };
	const float zero[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	float yaw_deg;

	madgwick_filter_init(&filter, 0.3f);
	madgwick_filter_reset(&filter, yaw_90);
	madgwick_filter_euler(&filter, NULL, NULL, &yaw_deg);
	expect_within(yaw_deg, 90.0f, ANGLE_TOLERANCE, "reset sets yaw");
	expect_close(filter.beta, 0.3f, "reset preserves beta");

	madgwick_filter_reset(&filter, not_finite);
	madgwick_filter_euler(&filter, NULL, NULL, &yaw_deg);
	expect_within(yaw_deg, 90.0f, ANGLE_TOLERANCE,
		      "non-finite reset ignored");

	madgwick_filter_reset(&filter, zero);
	madgwick_filter_euler(&filter, NULL, NULL, &yaw_deg);
	expect_within(yaw_deg, 90.0f, ANGLE_TOLERANCE, "zero reset ignored");

	madgwick_filter_reset(&filter, NULL);
	expect_quaternion(&filter, 1.0f, 0.0f, 0.0f, 0.0f, "NULL reset levels");

	madgwick_filter_reset(&filter, unnormalised);
	expect_quaternion(&filter, 1.0f, 0.0f, 0.0f, 0.0f,
			  "reset normalises");
}

static void test_quaternion_stays_unit(void)
{
	struct madgwick_filter filter;
	unsigned long seed = 1UL;
	float norm;
	int step;

	madgwick_filter_init(&filter, 0.1f);
	for (step = 0; step < 2000; step++)
		madgwick_filter_update(&filter,
				       jitter(&seed) * 5.0f,
				       jitter(&seed) * 5.0f,
				       jitter(&seed) * 5.0f,
				       jitter(&seed) * GRAVITY,
				       jitter(&seed) * GRAVITY,
				       jitter(&seed) * GRAVITY,
				       jitter(&seed), jitter(&seed),
				       jitter(&seed), 0.01f);

	norm = sqrtf(filter.q[0] * filter.q[0] + filter.q[1] * filter.q[1] +
		     filter.q[2] * filter.q[2] + filter.q[3] * filter.q[3]);
	expect_within(norm, 1.0f, 0.0001f, "norm after 2000 random updates");
}

static void test_stationary_stays_level(void)
{
	struct madgwick_filter filter;
	float roll_deg;
	float pitch_deg;
	int step;

	madgwick_filter_init(&filter, 0.1f);
	for (step = 0; step < 1000; step++)
		madgwick_filter_update_imu(&filter, 0.0f, 0.0f, 0.0f,
					   0.0f, 0.0f, GRAVITY, 0.01f);

	madgwick_filter_euler(&filter, &roll_deg, &pitch_deg, NULL);
	expect_within(roll_deg, 0.0f, ANGLE_TOLERANCE, "stationary roll");
	expect_within(pitch_deg, 0.0f, ANGLE_TOLERANCE, "stationary pitch");
}

/* Settle from level onto a fixed accelerometer reading and report the roll. */
static float settled_roll(float true_roll_deg, float beta, int steps)
{
	struct madgwick_filter filter;
	float accel[3];
	float roll_deg;
	int step;

	gravity_at_roll(true_roll_deg, accel);
	madgwick_filter_init(&filter, beta);
	for (step = 0; step < steps; step++)
		madgwick_filter_update_imu(&filter, 0.0f, 0.0f, 0.0f,
					   accel[0], accel[1], accel[2],
					   0.01f);

	madgwick_filter_euler(&filter, &roll_deg, NULL, NULL);

	return roll_deg;
}

static void test_roll_converges(void)
{
	float roll_deg;

	expect_within(settled_roll(20.0f, 0.1f, 2000), 20.0f, ANGLE_TOLERANCE,
		      "roll converges to 20 degrees");
	expect_within(settled_roll(-35.0f, 0.1f, 2000), -35.0f,
		      ANGLE_TOLERANCE, "roll converges to -35 degrees");

	/*
	 * Same case at the brief's beta 0.5. The gradient step is normalised,
	 * so the estimate settles into a limit cycle one step wide instead of
	 * converging; at beta 0.5 and dt 0.01 that step is about 0.54 degrees
	 * of roll, which is why the assertions above use the library default.
	 */
	roll_deg = settled_roll(20.0f, 0.5f, 2000);
	printf("note: beta 0.5 settles at %.3f degrees (limit cycle)\n",
	       (double)roll_deg);
	expect_within(roll_deg, 20.0f, 1.0f, "roll converges at beta 0.5");
}

static void test_gyro_only_yaw(void)
{
	struct madgwick_filter filter;
	float yaw_deg;
	int step;

	/* beta 0 ignores the accelerometer, so this is pure integration. */
	madgwick_filter_init(&filter, 0.0f);
	for (step = 0; step < 1000; step++)
		madgwick_filter_update_imu(&filter, 0.0f, 0.0f, 1.0f,
					   0.0f, 0.0f, GRAVITY, 0.001f);

	madgwick_filter_euler(&filter, NULL, NULL, &yaw_deg);
	expect_within(yaw_deg, RAD_TO_DEG, ANGLE_TOLERANCE,
		      "1 rad/s for 1 s is 1 radian of yaw");
}

static void test_invalid_input_keeps_state(void)
{
	struct madgwick_filter filter;

	madgwick_filter_init(&filter, 0.1f);
	madgwick_filter_update_imu(&filter, NAN, 0.0f, 0.0f,
				   0.0f, 0.0f, GRAVITY, 0.01f);
	expect_quaternion(&filter, 1.0f, 0.0f, 0.0f, 0.0f, "NaN gyro ignored");

	madgwick_filter_update_imu(&filter, 1.0f, 0.0f, 0.0f,
				   0.0f, 0.0f, INFINITY, 0.01f);
	expect_quaternion(&filter, 1.0f, 0.0f, 0.0f, 0.0f,
			  "infinite accelerometer ignored");

	madgwick_filter_update_imu(&filter, 1.0f, 0.0f, 0.0f,
				   0.0f, 0.0f, GRAVITY, 0.0f);
	expect_quaternion(&filter, 1.0f, 0.0f, 0.0f, 0.0f, "zero dt ignored");

	madgwick_filter_update_imu(&filter, 1.0f, 0.0f, 0.0f,
				   0.0f, 0.0f, GRAVITY, -0.01f);
	expect_quaternion(&filter, 1.0f, 0.0f, 0.0f, 0.0f,
			  "negative dt ignored");

	madgwick_filter_update_imu(&filter, 1.0f, 0.0f, 0.0f,
				   0.0f, 0.0f, GRAVITY, NAN);
	expect_quaternion(&filter, 1.0f, 0.0f, 0.0f, 0.0f, "NaN dt ignored");

	madgwick_filter_update(&filter, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, GRAVITY,
			       NAN, 0.0f, 0.0f, 0.01f);
	expect_quaternion(&filter, 1.0f, 0.0f, 0.0f, 0.0f,
			  "NaN magnetometer ignored");

	/* Free fall: no gravity to correct against, and the gyro is still. */
	madgwick_filter_update_imu(&filter, 0.0f, 0.0f, 0.0f,
				   0.0f, 0.0f, 0.0f, 0.01f);
	expect_quaternion(&filter, 1.0f, 0.0f, 0.0f, 0.0f,
			  "zero accelerometer skips the correction");
}

static void test_corrupt_state_is_repaired(void)
{
	struct madgwick_filter filter;

	madgwick_filter_init(&filter, 0.1f);
	filter.q[0] = NAN;
	madgwick_filter_update_imu(&filter, 0.0f, 0.0f, 0.0f,
				   0.0f, 0.0f, GRAVITY, 0.01f);
	expect_quaternion(&filter, 1.0f, 0.0f, 0.0f, 0.0f,
			  "non-finite quaternion repaired");

	filter.q[0] = 0.0f;
	filter.beta = -1.0f;
	madgwick_filter_update_imu(&filter, 0.0f, 0.0f, 0.0f,
				   0.0f, 0.0f, GRAVITY, 0.01f);
	expect_quaternion(&filter, 1.0f, 0.0f, 0.0f, 0.0f,
			  "zero quaternion repaired");
	expect_close(filter.beta, 0.1f, "negative beta repaired");
}

static void test_magnetometer_fixes_yaw(void)
{
	struct madgwick_filter filter;
	/* 30 degrees of yaw error to start from. */
	const float yaw_30[4] = { 0.96592583f, 0.0f, 0.0f, 0.25881905f };
	float roll_deg;
	float pitch_deg;
	float yaw_deg;
	int step;

	madgwick_filter_init(&filter, 0.1f);
	madgwick_filter_reset(&filter, yaw_30);

	/* Level and still, with a horizontal field pointing north. */
	for (step = 0; step < 5000; step++)
		madgwick_filter_update(&filter, 0.0f, 0.0f, 0.0f,
				       0.0f, 0.0f, GRAVITY,
				       1.0f, 0.0f, 0.0f, 0.01f);

	madgwick_filter_euler(&filter, &roll_deg, &pitch_deg, &yaw_deg);
	expect_within(yaw_deg, 0.0f, 1.0f, "magnetometer pulls yaw to north");
	expect_within(roll_deg, 0.0f, 1.0f, "magnetometer leaves roll level");
	expect_within(pitch_deg, 0.0f, 1.0f, "magnetometer leaves pitch level");
}

static void test_zero_magnetometer_falls_back(void)
{
	struct madgwick_filter with_mag;
	struct madgwick_filter imu_only;
	int step;

	madgwick_filter_init(&with_mag, 0.1f);
	madgwick_filter_init(&imu_only, 0.1f);

	for (step = 0; step < 100; step++) {
		madgwick_filter_update(&with_mag, 0.1f, -0.2f, 0.3f,
				       1.0f, 2.0f, GRAVITY,
				       0.0f, 0.0f, 0.0f, 0.01f);
		madgwick_filter_update_imu(&imu_only, 0.1f, -0.2f, 0.3f,
					   1.0f, 2.0f, GRAVITY, 0.01f);
	}

	expect_quaternion(&with_mag, imu_only.q[0], imu_only.q[1],
			  imu_only.q[2], imu_only.q[3],
			  "zero magnetometer matches the IMU path");
}

int main(void)
{
	test_null_handling();
	test_init();
	test_reset();
	test_quaternion_stays_unit();
	test_stationary_stays_level();
	test_roll_converges();
	test_gyro_only_yaw();
	test_invalid_input_keeps_state();
	test_corrupt_state_is_repaired();
	test_magnetometer_fixes_yaw();
	test_zero_magnetometer_falls_back();

	printf("PASS: %d checks\n", tests_run);
	return EXIT_SUCCESS;
}
