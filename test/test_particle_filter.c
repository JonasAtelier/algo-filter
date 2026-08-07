#include <math.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>

#include "particle_filter.h"

#define PARTICLE_COUNT	200U
#define TOLERANCE	0.0001f

static void fail(const char *message)
{
	fprintf(stderr, "FAIL: %s\n", message);
	exit(EXIT_FAILURE);
}

static void expect_close(float actual, float expected, float tolerance,
			 const char *message)
{
	if (!isfinite(actual) || fabsf(actual - expected) > tolerance)
		fail(message);
}

static void test_invalid_arguments(void)
{
	struct particle_filter filter;
	float particles[2];
	float weights[2];
	float scratch[2];

	if (particle_filter_init(NULL, particles, weights, scratch, 2U, 0.0f,
				 1.0f, 1.0f, 1.0f, 1U) == 0)
		fail("init accepted a null filter");
	if (particle_filter_init(&filter, particles, weights, scratch, 1U,
				 0.0f, 1.0f, 1.0f, 1.0f, 1U) == 0)
		fail("init accepted fewer than two particles");
	if (particle_filter_init(&filter, particles, weights, scratch, 2U,
				 0.0f, -1.0f, 1.0f, 1.0f, 1U) == 0)
		fail("init accepted a negative spread");
	if (particle_filter_init(&filter, particles, weights, scratch, 2U,
				 0.0f, 1.0f, 1.0f, 0.0f, 1U) == 0)
		fail("init accepted zero measurement noise");
	if (particle_filter_init(&filter, particles, particles, scratch, 2U,
				 0.0f, 1.0f, 1.0f, 1.0f, 1U) == 0)
		fail("init accepted overlapping storage");
}

static void test_exact_prediction(void)
{
	struct particle_filter filter;
	float particles[PARTICLE_COUNT];
	float weights[PARTICLE_COUNT];
	float scratch[PARTICLE_COUNT];
	size_t index;

	if (particle_filter_init(&filter, particles, weights, scratch,
				 PARTICLE_COUNT, 10.0f, 0.0f, 0.0f, 1.0f,
				 7U))
		fail("could not initialize exact prediction test");
	if (particle_filter_predict(&filter, 2.5f))
		fail("exact prediction failed");

	expect_close(particle_filter_get_estimate(&filter), 12.5f, TOLERANCE,
		     "prediction did not apply movement");
	for (index = 0; index < PARTICLE_COUNT; index++)
		expect_close(particles[index], 12.5f, TOLERANCE,
			     "zero process noise changed a particle");
}

static void test_measurement_moves_estimate(void)
{
	struct particle_filter filter;
	float particles[PARTICLE_COUNT];
	float weights[PARTICLE_COUNT];
	float scratch[PARTICLE_COUNT];
	float before;
	float after;

	if (particle_filter_init(&filter, particles, weights, scratch,
				 PARTICLE_COUNT, 0.0f, 5.0f, 0.2f, 0.01f,
				 42U))
		fail("could not initialize measurement test");

	before = particle_filter_get_estimate(&filter);
	if (particle_filter_update(&filter, 4.0f))
		fail("measurement update failed");
	after = particle_filter_get_estimate(&filter);

	if (!(after > before && fabsf(4.0f - after) < fabsf(4.0f - before)))
		fail("measurement did not move estimate toward observation");
	expect_close(particle_filter_get_effective_count(&filter),
		     (float)PARTICLE_COUNT, 0.1f,
		     "concentrated weights were not resampled");
}

static void test_reset_and_far_measurement(void)
{
	struct particle_filter filter;
	float particles[PARTICLE_COUNT];
	float weights[PARTICLE_COUNT];
	float scratch[PARTICLE_COUNT];
	float estimate;

	if (particle_filter_init(&filter, particles, weights, scratch,
				 PARTICLE_COUNT, 1.0f, 2.0f, 0.0f, 0.01f,
				 99U))
		fail("could not initialize reset test");
	if (particle_filter_reset(&filter, -3.0f, 0.0f))
		fail("reset failed");
	expect_close(particle_filter_get_estimate(&filter), -3.0f, TOLERANCE,
		     "reset did not replace the estimate");

	if (particle_filter_update(&filter, 1000000.0f))
		fail("far measurement caused a numerical failure");
	estimate = particle_filter_get_estimate(&filter);
	if (!isfinite(estimate))
		fail("far measurement produced a non-finite estimate");
}

static void test_extreme_finite_values(void)
{
	struct particle_filter filter;
	float particles[PARTICLE_COUNT];
	float weights[PARTICLE_COUNT];
	float scratch[PARTICLE_COUNT];
	struct particle_filter small_filter;
	float small_particles[3];
	float small_weights[3];
	float small_scratch[3];
	float before;

	if (particle_filter_init(&filter, particles, weights, scratch,
				 PARTICLE_COUNT, 1.0f, 0.0f, 0.0f,
				 1.0e-30f, 123U))
		fail("could not initialize tiny-noise test");
	if (particle_filter_update(&filter, 1.0f))
		fail("tiny noise rejected an exact finite measurement");

	if (particle_filter_reset(&filter, FLT_MAX, 0.0f))
		fail("could not reset to a finite extreme value");
	before = particles[0];
	if (particle_filter_predict(&filter, FLT_MAX) == 0)
		fail("prediction accepted state overflow");
	expect_close(particles[0], before, 0.0f,
		     "failed prediction changed particle state");
	expect_close(particle_filter_get_estimate(&filter), FLT_MAX, 0.0f,
		     "failed prediction changed estimate");

	if (particle_filter_init(&small_filter, small_particles, small_weights,
				 small_scratch, 3U, FLT_MAX, 0.0f, 0.0f,
				 1.0f, 456U))
		fail("could not initialize three-particle extreme test");
	if (particle_filter_predict(&small_filter, 0.0f))
		fail("rounded weights broke a no-op prediction");
	if (particle_filter_update(&small_filter, FLT_MAX))
		fail("rounded weights broke an exact measurement");
	expect_close(particle_filter_get_estimate(&small_filter), FLT_MAX,
		     0.0f, "extreme estimate changed unexpectedly");
	expect_close(particle_filter_get_effective_count(&small_filter), 3.0f,
		     TOLERANCE, "effective count exceeded particle count");
}

static void test_estimate_matches_particles(void)
{
	struct particle_filter filter;
	float particles[PARTICLE_COUNT];
	float weights[PARTICLE_COUNT];
	float scratch[PARTICLE_COUNT];
	double expected = 0.0;
	size_t index;

	if (particle_filter_init(&filter, particles, weights, scratch,
				 PARTICLE_COUNT, 3.0f, 4.0f, 0.0f, 0.01f,
				 17U))
		fail("could not initialize estimate-invariant test");

	for (index = 0; index < PARTICLE_COUNT; index++)
		expected += (double)particles[index] * weights[index];
	expect_close(particle_filter_get_estimate(&filter), (float)expected,
		     TOLERANCE, "initial estimate does not match particles");

	if (particle_filter_update(&filter, 8.0f))
		fail("estimate-invariant update failed");
	expected = 0.0;
	for (index = 0; index < PARTICLE_COUNT; index++)
		expected += (double)particles[index] * weights[index];
	expect_close(particle_filter_get_estimate(&filter), (float)expected,
		     TOLERANCE, "resampled estimate does not match particles");
}

static void test_invalid_runtime_input(void)
{
	struct particle_filter filter;
	float particles[2];
	float weights[2];
	float scratch[2];

	if (particle_filter_init(&filter, particles, weights, scratch, 2U,
				 0.0f, 0.0f, 0.0f, 1.0f, 1U))
		fail("could not initialize runtime-input test");
	if (particle_filter_predict(&filter, NAN) == 0)
		fail("predict accepted a non-finite movement");
	if (particle_filter_update(&filter, INFINITY) == 0)
		fail("update accepted a non-finite measurement");
	if (particle_filter_reset(&filter, 0.0f, -1.0f) == 0)
		fail("reset accepted a negative spread");
}

int main(void)
{
	test_invalid_arguments();
	test_exact_prediction();
	test_measurement_moves_estimate();
	test_reset_and_far_measurement();
	test_extreme_finite_values();
	test_estimate_matches_particles();
	test_invalid_runtime_input();

	puts("PASS: particle filter host tests");
	return EXIT_SUCCESS;
}
