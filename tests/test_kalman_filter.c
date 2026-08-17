#include "kalman_filter.h"

#include <float.h>
#include <math.h>
#include <stdio.h>

static int failures;

#define EXPECT_TRUE(condition)						\
	do {								\
		if (!(condition)) {					\
			fprintf(stderr, "%s:%d: %s\n", __FILE__,	\
				__LINE__, #condition);			\
			failures++;					\
		}							\
	} while (0)

#define EXPECT_STATUS(expected, expression)				\
	EXPECT_TRUE((expression) == (expected))

#define EXPECT_NEAR(expected, actual, tolerance)				\
	EXPECT_TRUE(fabsf((expected) - (actual)) <= (tolerance))

static int filter_equals(const struct kalman_filter *left,
			 const struct kalman_filter *right)
{
	return left->estimate == right->estimate &&
	       left->covariance == right->covariance &&
	       left->process_noise == right->process_noise &&
	       left->measurement_noise == right->measurement_noise &&
	       left->marker == right->marker &&
	       left->has_estimate == right->has_estimate;
}

static void test_default_workflow(void)
{
	struct kalman_filter filter;
	float result;

	EXPECT_STATUS(KALMAN_FILTER_OK, kalman_filter_init(&filter));
	EXPECT_STATUS(KALMAN_FILTER_OK,
		      kalman_filter_update(&filter, 10.0f, &result));
	EXPECT_TRUE(result == 10.0f);
	EXPECT_STATUS(KALMAN_FILTER_OK,
		      kalman_filter_update(&filter, 12.0f, &result));
	EXPECT_NEAR(11.004975f, result, 0.00001f);
	EXPECT_NEAR(0.5024876f, filter.covariance, 0.000001f);
	EXPECT_STATUS(KALMAN_FILTER_OK,
		      kalman_filter_value(&filter, &result));
	EXPECT_TRUE(result == filter.estimate);
}

static void test_convergence(void)
{
	struct kalman_filter filter;
	float result = 0.0f;
	int i;

	EXPECT_STATUS(KALMAN_FILTER_OK,
		      kalman_filter_init_with_noise(&filter, 0.01f, 1.0f));
	EXPECT_STATUS(KALMAN_FILTER_OK,
		      kalman_filter_update(&filter, 0.0f, &result));

	for (i = 0; i < 100; i++)
		EXPECT_STATUS(KALMAN_FILTER_OK,
			      kalman_filter_update(&filter, 10.0f, &result));

	EXPECT_TRUE(result > 9.9f && result < 10.0f);
	EXPECT_TRUE(filter.covariance >= 0.0f);
}

static void test_reset_and_tuning(void)
{
	struct kalman_filter filter;
	float result;

	EXPECT_STATUS(KALMAN_FILTER_OK, kalman_filter_init(&filter));
	EXPECT_STATUS(KALMAN_FILTER_OK,
		      kalman_filter_update(&filter, 5.0f, &result));
	EXPECT_STATUS(KALMAN_FILTER_OK,
		      kalman_filter_set_noise(&filter, 0.0f, 2.0f));
	EXPECT_STATUS(KALMAN_FILTER_OK, kalman_filter_reset(&filter));
	EXPECT_STATUS(KALMAN_FILTER_ERR_NOT_INITIALIZED,
		      kalman_filter_value(&filter, &result));
	EXPECT_STATUS(KALMAN_FILTER_OK,
		      kalman_filter_update(&filter, 42.0f, &result));
	EXPECT_TRUE(result == 42.0f);
}

static void test_invalid_arguments(void)
{
	struct kalman_filter filter;
	struct kalman_filter original;
	float result = 123.0f;

	EXPECT_STATUS(KALMAN_FILTER_ERR_NULL, kalman_filter_init(NULL));
	EXPECT_STATUS(KALMAN_FILTER_ERR_INVALID_VALUE,
		      kalman_filter_init_with_noise(&filter, -1.0f, 1.0f));
	EXPECT_STATUS(KALMAN_FILTER_ERR_INVALID_VALUE,
		      kalman_filter_init_with_noise(&filter, 1.0f, 0.0f));
	EXPECT_STATUS(KALMAN_FILTER_ERR_NOT_FINITE,
		      kalman_filter_init_with_noise(&filter, NAN, 1.0f));
	EXPECT_STATUS(KALMAN_FILTER_OK, kalman_filter_init(&filter));

	original = filter;
	EXPECT_STATUS(KALMAN_FILTER_ERR_NOT_FINITE,
		      kalman_filter_update(&filter, INFINITY, &result));
	EXPECT_TRUE(filter_equals(&filter, &original));
	EXPECT_TRUE(result == 123.0f);

	EXPECT_STATUS(KALMAN_FILTER_ERR_NULL,
		      kalman_filter_update(&filter, 1.0f, NULL));
	EXPECT_STATUS(KALMAN_FILTER_ERR_INVALID_VALUE,
		      kalman_filter_set_noise(&filter, 1.0f, -1.0f));
	EXPECT_TRUE(filter_equals(&filter, &original));
	EXPECT_STATUS(KALMAN_FILTER_ERR_INVALID_VALUE,
		      kalman_filter_update(&filter, 1.0f,
					   &filter.estimate));
	EXPECT_TRUE(filter_equals(&filter, &original));
	EXPECT_STATUS(KALMAN_FILTER_ERR_INVALID_VALUE,
		      kalman_filter_value(&filter, &filter.covariance));
	EXPECT_TRUE(filter_equals(&filter, &original));
}

static void test_invalid_state(void)
{
	struct kalman_filter filter = { 0 };
	float result;

	EXPECT_STATUS(KALMAN_FILTER_ERR_NOT_INITIALIZED,
		      kalman_filter_update(&filter, 1.0f, &result));

	EXPECT_STATUS(KALMAN_FILTER_OK, kalman_filter_init(&filter));
	filter.covariance = -1.0f;
	EXPECT_STATUS(KALMAN_FILTER_ERR_NOT_INITIALIZED,
		      kalman_filter_update(&filter, 1.0f, &result));
}

static void test_extreme_finite_values(void)
{
	struct kalman_filter filter;
	float result = 7.0f;

	EXPECT_STATUS(KALMAN_FILTER_OK,
		      kalman_filter_init_with_noise(&filter, FLT_MAX,
						    FLT_MAX));
	EXPECT_STATUS(KALMAN_FILTER_OK,
		      kalman_filter_update(&filter, 1.0f, &result));
	EXPECT_STATUS(KALMAN_FILTER_OK,
		      kalman_filter_update(&filter, -FLT_MAX, &result));
	EXPECT_TRUE(isfinite(result));
	EXPECT_TRUE(filter.covariance >= 0.0f);
}

int main(void)
{
	test_default_workflow();
	test_convergence();
	test_reset_and_tuning();
	test_invalid_arguments();
	test_invalid_state();
	test_extreme_finite_values();

	if (failures) {
		fprintf(stderr, "%d test(s) failed\n", failures);
		return 1;
	}

	puts("All C tests passed");
	return 0;
}
