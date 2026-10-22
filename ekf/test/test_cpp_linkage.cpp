/* The header claims C++ can use it. Prove the extern "C" block works. */
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "ekf_filter.h"

static void model(const float *x, const float *u, float dt, float *x_next,
		  void *ctx)
{
	(void)u;
	(void)ctx;
	x_next[0] = x[0] + x[1] * dt;
	x_next[1] = x[1];
}

static void measure(const float *x, float *z, void *ctx)
{
	(void)ctx;
	z[0] = x[0];
}

int main()
{
	ekf_filter filter;
	ekf_sensor sensor = ekf_sensor();
	const float x0[2] = { 0.0f, 0.0f };
	const float P0[4] = { 10.0f, 0.0f, 0.0f, 10.0f };
	const float Q[4] = { 1e-6f, 0.0f, 0.0f, 1e-6f };
	const float R[1] = { 0.05f };
	float z[1];
	int step;

	if (ekf_filter_init(&filter, 2, 1, x0, P0, Q, R) != EKF_FILTER_OK)
		return EXIT_FAILURE;

	filter.f = model;
	filter.h = measure;

	/* The same sensor again as an ekf_sensor, on every other step. */
	sensor.m = 1;
	sensor.h = measure;
	sensor.R[0] = 0.05f;

	/* A target moving at 2.0 per second, sampled cleanly at 10 Hz. */
	for (step = 1; step <= 200; step++) {
		z[0] = 2.0f * 0.1f * static_cast<float>(step);
		if (ekf_filter_predict(&filter, 0, 0.1f) != EKF_FILTER_OK)
			return EXIT_FAILURE;
		if ((step % 2 ? ekf_filter_update(&filter, z, 0) :
				ekf_filter_update_sensor(&filter, &sensor, z, 0)) !=
		    EKF_FILTER_OK)
			return EXIT_FAILURE;
	}

	if (std::fabs(filter.x[1] - 2.0f) > 0.01f)
		return EXIT_FAILURE;

	std::puts("PASS: C++ linkage smoke test");
	return EXIT_SUCCESS;
}
