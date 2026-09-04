/*
 * A simulated pendulum, so this runs anywhere with no hardware. The angle is
 * measured 100 times a second with noise on it; the rate is never measured at
 * all. The filter recovers the rate from how the angle moves, using the same
 * nonlinear model the simulation runs.
 *
 * No Jacobians are supplied, so the filter differentiates the model itself.
 */
#include "ekf_filter.h"

#include <math.h>
#include <stdio.h>

#define DT		0.01f	/* 100 Hz */
#define STEPS		300
#define PRINT_EVERY	10	/* every 100 ms */
#define GRAVITY		9.81f
#define LENGTH		1.0f
#define START_ANGLE	0.5f	/* radians */
#define ANGLE_NOISE	0.05f	/* radians, uniform, plus or minus */

/* theta'' = -(g / l) sin theta, one semi-implicit Euler step. Rate first, then
 * the angle with the rate that just came out, which is what keeps the energy
 * from creeping up over a long run. */
static void pendulum(const float *x, const float *u, float dt, float *x_next,
		     void *ctx)
{
	float rate;

	(void)u;
	(void)ctx;
	rate = x[1] - (GRAVITY / LENGTH) * sinf(x[0]) * dt;
	x_next[0] = x[0] + rate * dt;
	x_next[1] = rate;
}

static void measure_angle(const float *x, float *z, void *ctx)
{
	(void)ctx;
	z[0] = x[0];
}

static float jitter(unsigned long *seed)
{
	*seed = *seed * 1103515245UL + 12345UL;

	return (float)((*seed >> 16) % 2000UL) / 1000.0f - 1.0f;
}

int main(void)
{
	struct ekf_filter filter;
	const float x0[2] = { 0.0f, 0.0f };	/* the filter starts knowing nothing */
	const float P0[4] = { 1.0f, 0.0f, 0.0f, 1.0f };
	const float Q[4] = { 1e-8f, 0.0f, 0.0f, 1e-6f };
	const float R[1] = { 1e-3f };
	float truth[2] = { START_ANGLE, 0.0f };
	float next[2];
	float z[1];
	unsigned long seed = 2024UL;
	int step;

	if (ekf_filter_init(&filter, 2, 1, x0, P0, Q, R) != EKF_FILTER_OK)
		return 1;

	filter.f = pendulum;
	filter.h = measure_angle;

	puts(" time   true rate   estimated   error");
	for (step = 1; step <= STEPS; step++) {
		pendulum(truth, NULL, DT, next, NULL);
		truth[0] = next[0];
		truth[1] = next[1];

		z[0] = truth[0] + ANGLE_NOISE * jitter(&seed);

		if (ekf_filter_predict(&filter, NULL, DT) != EKF_FILTER_OK)
			return 1;
		if (ekf_filter_update(&filter, z, NULL) != EKF_FILTER_OK)
			return 1;

		if (step % PRINT_EVERY == 0)
			printf("%5.2fs %11.3f %11.3f %7.3f\n",
			       (double)((float)step * DT), (double)truth[1],
			       (double)filter.x[1],
			       (double)(filter.x[1] - truth[1]));
	}

	puts("\nThe filter is given a noisy angle and nothing else. The rate is");
	puts("inferred from the model, whose Jacobian it works out numerically.");

	return 0;
}
