/* Self-check: assert-based, no framework. Run with `make check`. */
#include "rls_filter.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

/* Deterministic stand-in for measurement noise: repeatable, no rand(). */
static float wobble(int i)
{
	return 0.01f * sinf(12.9898f * (float)i);
}

/* Two regressors that stay independent, so theta is always identifiable. */
static float excite(int i)
{
	return 2.0f * sinf(0.7f * (float)i) + cosf(0.31f * (float)i);
}

static void test_init_rejects_bad_arguments(void)
{
	struct rls_filter filter;

	assert(rls_filter_init(NULL, 2, 1.0f, 1e4f) == RLS_FILTER_ERR_NULL);

	assert(rls_filter_init(&filter, 0, 1.0f, 1e4f) ==
	       RLS_FILTER_ERR_INVALID_VALUE);
	assert(rls_filter_init(&filter, RLS_FILTER_MAX_PARAMS + 1, 1.0f,
			       1e4f) == RLS_FILTER_ERR_INVALID_VALUE);
	assert(rls_filter_init(&filter, 2, 0.0f, 1e4f) ==
	       RLS_FILTER_ERR_INVALID_VALUE);
	assert(rls_filter_init(&filter, 2, 1.1f, 1e4f) ==
	       RLS_FILTER_ERR_INVALID_VALUE);
	assert(rls_filter_init(&filter, 2, 1.0f, 0.0f) ==
	       RLS_FILTER_ERR_INVALID_VALUE);

	assert(rls_filter_init(&filter, 2, (float)NAN, 1e4f) ==
	       RLS_FILTER_ERR_NOT_FINITE);
	assert(rls_filter_init(&filter, 2, 1.0f, (float)INFINITY) ==
	       RLS_FILTER_ERR_NOT_FINITE);

	/* The edges that are legal. */
	assert(rls_filter_init(&filter, 1, 1.0f, 1e4f) == RLS_FILTER_OK);
	assert(rls_filter_init(&filter, RLS_FILTER_MAX_PARAMS, 0.5f, 1e-6f) ==
	       RLS_FILTER_OK);
}

static void test_rejects_uninitialised(void)
{
	struct rls_filter filter = { 0 };
	const float phi[2] = { 1.0f, 1.0f };

	assert(rls_filter_update(&filter, phi, 1.0f, NULL) ==
	       RLS_FILTER_ERR_NOT_INITIALIZED);
	assert(rls_filter_reset(&filter, 1e4f) ==
	       RLS_FILTER_ERR_NOT_INITIALIZED);

	assert(rls_filter_update(NULL, phi, 1.0f, NULL) ==
	       RLS_FILTER_ERR_NULL);
	assert(rls_filter_reset(NULL, 1e4f) == RLS_FILTER_ERR_NULL);

	assert(rls_filter_init(&filter, 2, 1.0f, 1e4f) == RLS_FILTER_OK);
	assert(rls_filter_update(&filter, NULL, 1.0f, NULL) ==
	       RLS_FILTER_ERR_NULL);
}

/* The whole point of the library: recover a line from samples of it. */
static void test_fits_a_line(void)
{
	struct rls_filter filter;
	int i;

	assert(rls_filter_init(&filter, 2, 1.0f, 1e4f) == RLS_FILTER_OK);

	for (i = 0; i < 200; i++) {
		float x = excite(i);
		float phi[2];
		float error = 0.0f;

		phi[0] = x;
		phi[1] = 1.0f;

		assert(rls_filter_update(&filter, phi,
					 2.0f * x + 1.0f + wobble(i),
					 &error) == RLS_FILTER_OK);

		if (i >= 20)
			assert(fabs((double)error) < 0.05);
	}

	assert(fabs((double)filter.theta[0] - 2.0) < 0.02);
	assert(fabs((double)filter.theta[1] - 1.0) < 0.02);
}

/*
 * lambda < 1 lets old samples fade, so the estimate can follow a change.
 *
 * The influence of the stale data decays as lambda^k, so a jump of 3.0 needs
 * 3.0 * 0.95^k < 0.1, which is k > 66. 80 samples is that with some margin;
 * asking for 60 would be asking the filter to beat its own time constant.
 */
static void test_forgetting_tracks_a_change(void)
{
	struct rls_filter forgetful;
	struct rls_filter faithful;
	int i;

	assert(rls_filter_init(&forgetful, 2, 0.95f, 1e4f) == RLS_FILTER_OK);
	assert(rls_filter_init(&faithful, 2, 1.0f, 1e4f) == RLS_FILTER_OK);

	for (i = 0; i < 380; i++) {
		float x = excite(i);
		float slope = i < 300 ? 2.0f : -1.0f;
		float phi[2];

		phi[0] = x;
		phi[1] = 1.0f;

		assert(rls_filter_update(&forgetful, phi,
					 slope * x + 1.0f, NULL) ==
		       RLS_FILTER_OK);
		assert(rls_filter_update(&faithful, phi,
					 slope * x + 1.0f, NULL) ==
		       RLS_FILTER_OK);
	}

	/* 80 samples after the slope flipped from 2 to -1. */
	assert(fabs((double)forgetful.theta[0] + 1.0) < 0.1);

	/* lambda 1 weighs the 300 old samples as heavily as the new ones, so
	 * it is still most of the way back at 2. */
	assert(fabs((double)faithful.theta[0] + 1.0) >= 0.1);
}

/* A long run must not quietly corrupt the covariance. */
static void test_covariance_stays_sane(void)
{
	struct rls_filter filter;
	int i;
	int j;
	int k;

	assert(rls_filter_init(&filter, 3, 0.98f, 1e3f) == RLS_FILTER_OK);

	for (k = 0; k < 5000; k++) {
		float phi[3];
		float y;

		phi[0] = sinf(0.7f * (float)k);
		phi[1] = cosf(0.31f * (float)k);
		phi[2] = 1.0f;
		y = 1.5f * phi[0] - 0.5f * phi[1] + 2.0f * phi[2];

		assert(rls_filter_update(&filter, phi, y, NULL) ==
		       RLS_FILTER_OK);

		for (i = 0; i < 3; i++) {
			assert(filter.P[i * 3 + i] > 0.0f);

			for (j = 0; j < 3; j++)
				assert(fabs((double)filter.P[i * 3 + j] -
					    (double)filter.P[j * 3 + i]) <
				       1e-6);
		}
	}

	assert(fabs((double)filter.theta[0] - 1.5) < 0.01);
}

/* One bad sample must not touch a good estimate. */
static void test_rejects_non_finite_sample(void)
{
	struct rls_filter filter;
	float phi[2];
	float bad[2];
	float kept[2];
	float error = 12.0f;
	int i;

	assert(rls_filter_init(&filter, 2, 1.0f, 1e4f) == RLS_FILTER_OK);

	for (i = 0; i < 10; i++) {
		phi[0] = excite(i);
		phi[1] = 1.0f;
		assert(rls_filter_update(&filter, phi, 2.0f * phi[0] + 1.0f,
					 NULL) == RLS_FILTER_OK);
	}

	kept[0] = filter.theta[0];
	kept[1] = filter.theta[1];

	bad[0] = (float)NAN;
	bad[1] = 1.0f;
	assert(rls_filter_update(&filter, bad, 3.0f, &error) ==
	       RLS_FILTER_ERR_NOT_FINITE);

	bad[0] = (float)INFINITY;
	assert(rls_filter_update(&filter, bad, 3.0f, &error) ==
	       RLS_FILTER_ERR_NOT_FINITE);

	phi[0] = 1.0f;
	phi[1] = 1.0f;
	assert(rls_filter_update(&filter, phi, (float)NAN, &error) ==
	       RLS_FILTER_ERR_NOT_FINITE);
	assert(rls_filter_update(&filter, phi, (float)INFINITY, &error) ==
	       RLS_FILTER_ERR_NOT_FINITE);

	assert(filter.theta[0] == kept[0]);
	assert(filter.theta[1] == kept[1]);
	assert(error == 12.0f);		/* the output was left alone too */
}

static void test_predict(void)
{
	struct rls_filter filter;
	struct rls_filter zeroed = { 0 };
	float phi[2];
	double expected;

	assert(rls_filter_init(&filter, 2, 1.0f, 1e4f) == RLS_FILTER_OK);
	filter.theta[0] = 3.0f;
	filter.theta[1] = -0.5f;

	phi[0] = 2.0f;
	phi[1] = 4.0f;
	expected = 3.0 * 2.0 + -0.5 * 4.0;
	assert(fabs((double)rls_filter_predict(&filter, phi) - expected) <
	       1e-5);

	assert(isnan(rls_filter_predict(&zeroed, phi)));
	assert(isnan(rls_filter_predict(NULL, phi)));
	assert(isnan(rls_filter_predict(&filter, NULL)));

	phi[0] = (float)NAN;
	assert(isnan(rls_filter_predict(&filter, phi)));
}

static void test_reset(void)
{
	struct rls_filter filter;
	float phi[2];
	int i;
	int j;

	assert(rls_filter_init(&filter, 2, 0.97f, 1e4f) == RLS_FILTER_OK);

	for (i = 0; i < 30; i++) {
		phi[0] = excite(i);
		phi[1] = 1.0f;
		assert(rls_filter_update(&filter, phi, 2.0f * phi[0] + 1.0f,
					 NULL) == RLS_FILTER_OK);
	}

	assert(rls_filter_reset(&filter, 500.0f) == RLS_FILTER_OK);
	assert(filter.n == 2);
	assert(filter.lambda == 0.97f);

	for (i = 0; i < 2; i++) {
		assert(filter.theta[i] == 0.0f);

		for (j = 0; j < 2; j++)
			assert(filter.P[i * 2 + j] ==
			       (i == j ? 500.0f : 0.0f));
	}

	assert(rls_filter_reset(&filter, 0.0f) ==
	       RLS_FILTER_ERR_INVALID_VALUE);
	assert(rls_filter_reset(&filter, (float)NAN) ==
	       RLS_FILTER_ERR_NOT_FINITE);
}

int main(void)
{
	test_init_rejects_bad_arguments();
	test_rejects_uninitialised();
	test_fits_a_line();
	test_forgetting_tracks_a_change();
	test_covariance_stays_sane();
	test_rejects_non_finite_sample();
	test_predict();
	test_reset();
	printf("all tests passed\n");
	return 0;
}
