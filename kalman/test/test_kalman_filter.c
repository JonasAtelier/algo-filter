/* Self-check: assert-based, no framework. Run with `make check`. */
#include "kalman_filter.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void test_init_defaults(void)
{
	struct kalman_filter filter;

	assert(kalman_filter_init(&filter) == KALMAN_FILTER_OK);
	assert(filter.process_noise == KALMAN_FILTER_DEFAULT_PROCESS_NOISE);
	assert(filter.measurement_noise ==
	       KALMAN_FILTER_DEFAULT_MEASUREMENT_NOISE);
	assert(filter.has_estimate == 0);
}

/* The first sample has nothing to smooth against, so it is taken as-is. */
static void test_first_update_seeds(void)
{
	struct kalman_filter filter;
	float out = 0.0f;

	kalman_filter_init(&filter);
	assert(kalman_filter_update(&filter, 42.5f, &out) == KALMAN_FILTER_OK);
	assert(out == 42.5f);
	assert(filter.has_estimate == 1);
}

static void test_converges_to_constant(void)
{
	struct kalman_filter filter;
	float out = 0.0f;
	int i;

	kalman_filter_init(&filter);
	kalman_filter_update(&filter, 0.0f, &out);

	for (i = 0; i < 200; i++)
		assert(kalman_filter_update(&filter, 5.0f, &out) ==
		       KALMAN_FILTER_OK);

	assert(fabs((double)out - 5.0) < 1e-3);
}

/* Output must sit between the old estimate and the new measurement. */
static void test_smooths(void)
{
	struct kalman_filter filter;
	float out = 0.0f;

	kalman_filter_init(&filter);
	kalman_filter_update(&filter, 0.0f, &out);
	kalman_filter_update(&filter, 10.0f, &out);
	assert(out > 0.0f && out < 10.0f);
}

/* Trusting the sensor more must track a step faster. */
static void test_noise_tuning_changes_response(void)
{
	struct kalman_filter trusting;
	struct kalman_filter doubting;
	float fast = 0.0f;
	float slow = 0.0f;

	assert(kalman_filter_init_with_noise(&trusting, 0.01f, 0.1f) ==
	       KALMAN_FILTER_OK);
	assert(kalman_filter_init_with_noise(&doubting, 0.01f, 100.0f) ==
	       KALMAN_FILTER_OK);

	kalman_filter_update(&trusting, 0.0f, &fast);
	kalman_filter_update(&doubting, 0.0f, &slow);
	kalman_filter_update(&trusting, 10.0f, &fast);
	kalman_filter_update(&doubting, 10.0f, &slow);

	assert(fast > slow);
}

static void test_reset_reseeds(void)
{
	struct kalman_filter filter;
	float out = 0.0f;

	kalman_filter_init(&filter);
	kalman_filter_update(&filter, 3.0f, &out);
	kalman_filter_update(&filter, 3.0f, &out);

	assert(kalman_filter_reset(&filter) == KALMAN_FILTER_OK);
	assert(filter.has_estimate == 0);

	assert(kalman_filter_update(&filter, 99.0f, &out) == KALMAN_FILTER_OK);
	assert(out == 99.0f);		/* seeded again, no pull from the past */
}

static void test_rejects_bad_noise(void)
{
	struct kalman_filter filter;

	assert(kalman_filter_init_with_noise(&filter, 0.01f, 0.0f) ==
	       KALMAN_FILTER_ERR_INVALID_VALUE);
	assert(kalman_filter_init_with_noise(&filter, -1.0f, 1.0f) ==
	       KALMAN_FILTER_ERR_INVALID_VALUE);
	assert(kalman_filter_init_with_noise(&filter, (float)NAN, 1.0f) ==
	       KALMAN_FILTER_ERR_NOT_FINITE);
	assert(kalman_filter_init_with_noise(&filter, 0.01f, (float)INFINITY) ==
	       KALMAN_FILTER_ERR_NOT_FINITE);
}

static void test_rejects_null(void)
{
	struct kalman_filter filter;
	float out = 0.0f;

	kalman_filter_init(&filter);
	assert(kalman_filter_init(NULL) == KALMAN_FILTER_ERR_NULL);
	assert(kalman_filter_init_with_noise(NULL, 0.01f, 1.0f) ==
	       KALMAN_FILTER_ERR_NULL);
	assert(kalman_filter_reset(NULL) == KALMAN_FILTER_ERR_NULL);
	assert(kalman_filter_update(NULL, 1.0f, &out) ==
	       KALMAN_FILTER_ERR_NULL);
	assert(kalman_filter_update(&filter, 1.0f, NULL) ==
	       KALMAN_FILTER_ERR_NULL);
}

static void test_rejects_uninitialised(void)
{
	struct kalman_filter filter = KALMAN_FILTER_INITIALIZER;
	float out = 0.0f;

	assert(kalman_filter_update(&filter, 1.0f, &out) ==
	       KALMAN_FILTER_ERR_NOT_INITIALIZED);
	assert(kalman_filter_reset(&filter) ==
	       KALMAN_FILTER_ERR_NOT_INITIALIZED);
}

/* A bad sample must not corrupt a good estimate. */
static void test_rejects_non_finite_measurement(void)
{
	struct kalman_filter filter;
	float out = 0.0f;
	float kept;

	kalman_filter_init(&filter);
	kalman_filter_update(&filter, 7.0f, &out);
	kept = filter.estimate;

	assert(kalman_filter_update(&filter, (float)NAN, &out) ==
	       KALMAN_FILTER_ERR_NOT_FINITE);
	assert(kalman_filter_update(&filter, (float)INFINITY, &out) ==
	       KALMAN_FILTER_ERR_NOT_FINITE);
	assert(filter.estimate == kept);
	assert(out == 7.0f);		/* output left untouched too */
}

/* Writing the result into the filter itself would corrupt it mid-update. */
static void test_rejects_aliased_result(void)
{
	struct kalman_filter filter;

	kalman_filter_init(&filter);
	assert(kalman_filter_update(&filter, 1.0f, &filter.estimate) ==
	       KALMAN_FILTER_ERR_INVALID_VALUE);
	assert(kalman_filter_update(&filter, 1.0f, &filter.covariance) ==
	       KALMAN_FILTER_ERR_INVALID_VALUE);
}

static void test_instances_are_independent(void)
{
	struct kalman_filter a;
	struct kalman_filter b;
	float out = 0.0f;

	kalman_filter_init(&a);
	kalman_filter_init(&b);
	kalman_filter_update(&a, 100.0f, &out);
	assert(b.has_estimate == 0);
	assert(b.estimate == 0.0f);
}

int main(void)
{
	test_init_defaults();
	test_first_update_seeds();
	test_converges_to_constant();
	test_smooths();
	test_noise_tuning_changes_response();
	test_reset_reseeds();
	test_rejects_bad_noise();
	test_rejects_null();
	test_rejects_uninitialised();
	test_rejects_non_finite_measurement();
	test_rejects_aliased_result();
	test_instances_are_independent();
	printf("all tests passed\n");
	return 0;
}
