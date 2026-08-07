#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "complementary_filter.h"

#define FLOAT_TOLERANCE	0.0001f

static int tests_run;

static void expect_close(float actual, float expected, const char *name)
{
	tests_run++;
	if (isfinite(actual) &&
	    fabsf(actual - expected) <= FLOAT_TOLERANCE)
		return;

	fprintf(stderr, "FAIL: %s: expected %.6f, got %.6f\n",
		name, expected, actual);
	exit(EXIT_FAILURE);
}

static void expect_finite(float actual, const char *name)
{
	tests_run++;
	if (isfinite(actual))
		return;

	fprintf(stderr, "FAIL: %s: expected a finite value, got %.6f\n",
		name, actual);
	exit(EXIT_FAILURE);
}

static void test_null_handling(void)
{
	complementary_filter_init(NULL, 10.0f, 0.5f);
	complementary_filter_reset(NULL, 20.0f);
	expect_close(complementary_filter_update(NULL, 1.0f, 2.0f, 0.1f),
		     0.0f, "null update");
}

static void test_alpha_validation(void)
{
	struct complementary_filter filter;

	complementary_filter_init(&filter, 0.0f, -0.5f);
	expect_close(filter.alpha, 0.0f, "low alpha clamp");

	complementary_filter_init(&filter, 0.0f, 1.5f);
	expect_close(filter.alpha, 1.0f, "high alpha clamp");

	complementary_filter_init(&filter, 0.0f, NAN);
	expect_close(filter.alpha, 0.98f, "NaN alpha default");

	complementary_filter_init(&filter, 0.0f, INFINITY);
	expect_close(filter.alpha, 0.98f, "infinite alpha default");

	filter.alpha = -2.0f;
	complementary_filter_update(&filter, 0.0f, 20.0f, 1.0f);
	expect_close(filter.alpha, 0.0f, "public low alpha clamp");
	expect_close(filter.angle_deg, 20.0f, "public low alpha behavior");

	filter.alpha = 2.0f;
	complementary_filter_update(&filter, 5.0f, 99.0f, 1.0f);
	expect_close(filter.alpha, 1.0f, "public high alpha clamp");
	expect_close(filter.angle_deg, 25.0f, "public high alpha behavior");

	filter.alpha = -INFINITY;
	complementary_filter_update(&filter, 0.0f, 25.0f, 1.0f);
	expect_close(filter.alpha, 0.98f, "public infinite alpha default");
}

static void test_init_and_reset(void)
{
	struct complementary_filter filter;

	complementary_filter_init(&filter, 12.5f, 0.75f);
	expect_close(filter.angle_deg, 12.5f, "initial angle");
	expect_close(filter.alpha, 0.75f, "initial alpha");

	complementary_filter_reset(&filter, -8.0f);
	expect_close(filter.angle_deg, -8.0f, "reset angle");
	expect_close(filter.alpha, 0.75f, "reset preserves alpha");

	complementary_filter_reset(&filter, NAN);
	expect_close(filter.angle_deg, -8.0f, "NaN reset preserves angle");

	filter.angle_deg = INFINITY;
	complementary_filter_reset(&filter, -INFINITY);
	expect_close(filter.angle_deg, 0.0f, "invalid reset repairs state");

	complementary_filter_init(&filter, NAN, 0.5f);
	expect_close(filter.angle_deg, 0.0f, "NaN initial angle");

	complementary_filter_init(&filter, -INFINITY, 0.5f);
	expect_close(filter.angle_deg, 0.0f, "infinite initial angle");
}

static void test_alpha_endpoints(void)
{
	struct complementary_filter filter;
	float angle;

	complementary_filter_init(&filter, 10.0f, 1.0f);
	angle = complementary_filter_update(&filter, 20.0f, FLT_MAX, 0.5f);
	expect_close(angle, 20.0f, "gyro-only update");

	complementary_filter_init(&filter, 10.0f, 0.0f);
	angle = complementary_filter_update(&filter, FLT_MAX, 30.0f, FLT_MAX);
	expect_close(angle, 30.0f, "accelerometer endpoint avoids overflow");

	complementary_filter_init(&filter, 179.0f, 0.0f);
	angle = complementary_filter_update(&filter, 0.0f, -179.0f, 1.0f);
	expect_close(angle, 181.0f, "accelerometer endpoint boundary crossing");
}

static void test_regular_updates(void)
{
	struct complementary_filter filter;
	float angle;

	complementary_filter_init(&filter, 10.0f, 0.8f);
	angle = complementary_filter_update(&filter, 5.0f, 20.0f, 2.0f);
	expect_close(angle, 20.0f, "mixed update");

	complementary_filter_init(&filter, -10.0f, 0.5f);
	angle = complementary_filter_update(&filter, -20.0f, -40.0f, 0.5f);
	expect_close(angle, -30.0f, "negative values");

	complementary_filter_init(&filter, 0.0f, 0.5f);
	angle = complementary_filter_update(&filter, 10.0f, 20.0f, 1.0f);
	expect_close(angle, 15.0f, "first repeated update");
	angle = complementary_filter_update(&filter, 10.0f, 20.0f, 1.0f);
	expect_close(angle, 22.5f, "second repeated update");
}

static void test_invalid_updates(void)
{
	struct complementary_filter filter;

	complementary_filter_init(&filter, 15.0f, 0.5f);
	expect_close(complementary_filter_update(&filter, 10.0f, 30.0f,
						0.0f),
		     15.0f, "zero dt");
	expect_close(complementary_filter_update(&filter, 10.0f, 30.0f,
						-1.0f),
		     15.0f, "negative dt");
	expect_close(complementary_filter_update(&filter, 10.0f, 30.0f,
						NAN),
		     15.0f, "NaN dt");
	expect_close(complementary_filter_update(&filter, 10.0f, 30.0f,
						INFINITY),
		     15.0f, "infinite dt");
	expect_close(complementary_filter_update(&filter, NAN, 30.0f, 1.0f),
		     15.0f, "NaN gyro");
	expect_close(complementary_filter_update(&filter, INFINITY, 30.0f,
						1.0f),
		     15.0f, "infinite gyro");
	expect_close(complementary_filter_update(&filter, 10.0f, NAN, 1.0f),
		     15.0f, "NaN accelerometer");
	expect_close(complementary_filter_update(&filter, 10.0f, -INFINITY,
						1.0f),
		     15.0f, "infinite accelerometer");
	expect_close(filter.angle_deg, 15.0f, "invalid inputs preserve angle");
}

static void test_public_state_repair(void)
{
	struct complementary_filter filter;
	float angle;

	complementary_filter_init(&filter, 10.0f, 0.5f);
	filter.angle_deg = NAN;
	angle = complementary_filter_update(&filter, 0.0f, 20.0f, 1.0f);
	expect_finite(angle, "NaN public angle repaired");
	expect_close(angle, 10.0f, "NaN public angle uses safe zero");

	filter.angle_deg = INFINITY;
	angle = complementary_filter_update(&filter, 0.0f, 20.0f, -1.0f);
	expect_close(angle, 0.0f, "invalid dt returns repaired angle");
}

static void test_overflow_preserves_angle(void)
{
	struct complementary_filter filter;

	complementary_filter_init(&filter, 42.0f, 0.5f);
	expect_close(complementary_filter_update(&filter, FLT_MAX, 0.0f,
						FLT_MAX),
		     42.0f, "gyro multiplication overflow");

	complementary_filter_init(&filter, FLT_MAX, 1.0f);
	expect_close(complementary_filter_update(&filter, FLT_MAX, 0.0f, 1.0f),
		     FLT_MAX, "gyro addition overflow");
	expect_finite(filter.angle_deg, "overflow leaves finite state");
}

static void test_angle_boundary(void)
{
	struct complementary_filter filter;
	float angle;

	complementary_filter_init(&filter, 179.0f, 0.5f);
	angle = complementary_filter_update(&filter, 0.0f, -179.0f, 1.0f);
	expect_close(angle, 180.0f, "positive boundary crossing");

	complementary_filter_init(&filter, -179.0f, 0.5f);
	angle = complementary_filter_update(&filter, 0.0f, 179.0f, 1.0f);
	expect_close(angle, -180.0f, "negative boundary crossing");

	complementary_filter_init(&filter, 179.0f, 0.5f);
	angle = complementary_filter_update(&filter, 4.0f, -179.0f, 0.5f);
	expect_close(angle, 181.0f, "gyro integration across boundary");
	angle = complementary_filter_update(&filter, 4.0f, -175.0f, 0.5f);
	expect_close(angle, 184.0f, "repeated boundary update");
}

int main(void)
{
	test_null_handling();
	test_alpha_validation();
	test_init_and_reset();
	test_alpha_endpoints();
	test_regular_updates();
	test_invalid_updates();
	test_public_state_repair();
	test_overflow_preserves_angle();
	test_angle_boundary();

	printf("PASS: %d checks\n", tests_run);
	return EXIT_SUCCESS;
}
