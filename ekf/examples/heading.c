/*
 * A simulated heading, so this runs anywhere with no hardware. A gyro with an
 * unknown bias drives the prediction from a loop whose period wanders between
 * 8 and 12 ms. A compass reads the heading at 50 Hz and now and then throws a
 * 30 degree spike; a slower, sharper course sensor reads it at 2 Hz. The
 * heading turns through +/- pi several times on the way.
 *
 * What it shows: the gyro as the input u, Q as a rate (q_per_second) so the
 * jitter does not change the tuning, a wrapped residual and a normalised state
 * so +/- pi is not a 2 pi jump, the NIS gate throwing the spikes out, and a
 * second sensor with its own R through struct ekf_sensor.
 */
#include "ekf_filter.h"

#include <math.h>
#include <stdio.h>

#define PI		3.14159265358979323846
#define DT_NOMINAL	0.010f	/* 100 Hz loop, plus or minus 2 ms */
#define DT_JITTER	0.002f
#define DURATION	20.0	/* seconds */
#define COMPASS_PERIOD	0.020	/* 50 Hz */
#define COURSE_PERIOD	0.500	/* 2 Hz */
#define SPIKE_PERIOD	2.0	/* one bad compass reading every 2 s */
#define SPIKE		0.52f	/* 30 degrees */
#define GYRO_BIAS	0.02f	/* rad/s, the filter is not told */
#define GYRO_NOISE	0.01f	/* rad/s, uniform, plus or minus */
#define COMPASS_NOISE	0.05f	/* rad, uniform, plus or minus */
#define COURSE_NOISE	0.01f
#define GATE		9.0f	/* NIS, 3 sigma for one measurement */

static double wrap(double angle)
{
	while (angle > PI)
		angle -= 2.0 * PI;
	while (angle <= -PI)
		angle += 2.0 * PI;

	return angle;
}

/* x = (heading, gyro bias), u = the gyro reading. */
static void gyro_model(const float *x, const float *u, float dt, float *x_next,
		       void *ctx)
{
	(void)ctx;
	x_next[0] = x[0] + (u[0] - x[1]) * dt;
	x_next[1] = x[1];
}

static void measure_heading(const float *x, float *z, void *ctx)
{
	(void)ctx;
	z[0] = x[0];
}

/* -3.1 measured against a predicted +3.1 is 0.08 rad, not -6.2. */
static void wrapped_residual(const float *z, const float *predicted, float *y,
			     void *ctx)
{
	(void)ctx;
	y[0] = (float)wrap((double)z[0] - (double)predicted[0]);
}

static void wrap_state(float *x, void *ctx)
{
	(void)ctx;
	x[0] = (float)wrap((double)x[0]);
}

static float jitter(unsigned long *seed)
{
	*seed = *seed * 1103515245UL + 12345UL;

	return (float)((*seed >> 16) % 2000UL) / 1000.0f - 1.0f;
}

int main(void)
{
	struct ekf_filter filter;
	struct ekf_sensor course = { 0 };
	const float x0[2] = { 0.0f, 0.0f };	/* heading and bias unknown */
	const float P0[4] = { 4.0f, 0.0f, 0.0f, 1e-3f };
	const float Q[4] = { 1e-4f, 0.0f, 0.0f, 1e-7f };	/* per second */
	const float R[1] = { 1e-3f };		/* compass */
	unsigned long seed = 1701UL;
	double t = 0.0, truth = 2.5, rate;
	double next_compass = COMPASS_PERIOD, next_course = COURSE_PERIOD;
	double next_spike = SPIKE_PERIOD, next_print = 1.0;
	int spikes = 0, rejected = 0, compass = 0, courses = 0;
	enum ekf_filter_status status;
	float u[1], z[1];
	float dt;

	if (ekf_filter_init(&filter, 2, 1, x0, P0, Q, R) != EKF_FILTER_OK)
		return 1;

	filter.f = gyro_model;
	filter.h = measure_heading;		/* the compass is the filter's own */
	filter.residual = wrapped_residual;
	filter.normalize = wrap_state;
	filter.gate = GATE;
	filter.q_per_second = 1;

	course.m = 1;
	course.h = measure_heading;
	course.residual = wrapped_residual;
	course.R[0] = 1e-4f;			/* sharper than the compass */

	puts(" time   true hdg   estimated   error   bias est");
	while (t < DURATION) {
		dt = DT_NOMINAL + DT_JITTER * jitter(&seed);
		rate = 0.5 + 0.3 * sin(t);
		truth = wrap(truth + rate * (double)dt);
		t += (double)dt;

		u[0] = (float)rate + GYRO_BIAS + GYRO_NOISE * jitter(&seed);
		if (ekf_filter_predict(&filter, u, dt) != EKF_FILTER_OK)
			return 1;

		if (t >= next_compass) {
			next_compass += COMPASS_PERIOD;
			z[0] = (float)wrap(truth + COMPASS_NOISE * jitter(&seed));
			if (t >= next_spike) {
				next_spike += SPIKE_PERIOD;
				z[0] = (float)wrap((double)z[0] + SPIKE);
				spikes++;
			}

			status = ekf_filter_update(&filter, z, NULL);
			if (status == EKF_FILTER_REJECTED)
				rejected++;
			else if (status != EKF_FILTER_OK)
				return 1;
			compass++;
		}

		if (t >= next_course) {
			next_course += COURSE_PERIOD;
			z[0] = (float)wrap(truth + COURSE_NOISE * jitter(&seed));
			if (ekf_filter_update_sensor(&filter, &course, z, NULL) !=
			    EKF_FILTER_OK)
				return 1;
			courses++;
		}

		if (t >= next_print) {
			next_print += 1.0;
			printf("%5.2fs %10.3f %11.3f %7.3f %10.4f\n", t, truth,
			       (double)filter.x[0],
			       wrap((double)filter.x[0] - truth),
			       (double)filter.x[1]);
		}
	}

	printf("\n%d compass readings, %d spikes injected, %d rejected by the gate;\n",
	       compass, spikes, rejected);
	printf("%d course readings. True gyro bias %.3f rad/s.\n", courses,
	       (double)GYRO_BIAS);

	return 0;
}
