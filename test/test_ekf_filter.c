/* Self-check: assert-free, exit-on-failure, no framework. Run with `make check`. */
#include "ekf_filter.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DT		0.1f
#define TRUE_VELOCITY	1.0f
#define MEAS_NOISE	0.5f	/* uniform, plus or minus this */

static int tests_run;

static void expect_within(double actual, double expected, double tolerance,
			  const char *name)
{
	tests_run++;
	if (isfinite(actual) && fabs(actual - expected) <= tolerance)
		return;

	fprintf(stderr, "FAIL: %s: expected %.6f +/- %.6f, got %.6f\n",
		name, expected, tolerance, actual);
	exit(EXIT_FAILURE);
}

static void expect_status(enum ekf_filter_status actual,
			  enum ekf_filter_status expected, const char *name)
{
	tests_run++;
	if (actual == expected)
		return;

	fprintf(stderr, "FAIL: %s: expected status %d, got %d\n", name,
		(int)expected, (int)actual);
	exit(EXIT_FAILURE);
}

static void expect_true(int condition, const char *name)
{
	tests_run++;
	if (condition)
		return;

	fprintf(stderr, "FAIL: %s\n", name);
	exit(EXIT_FAILURE);
}

/* Deterministic pseudo-random floats in [-1, 1], as the siblings use. */
static float jitter(unsigned long *seed)
{
	*seed = *seed * 1103515245UL + 12345UL;

	return (float)((*seed >> 16) % 2000UL) / 1000.0f - 1.0f;
}

/* ---- a constant-velocity tracker: state (position, velocity) ---------- */

static void cv_model(const float *x, const float *u, float dt, float *x_next,
		     void *ctx)
{
	(void)u;
	(void)ctx;
	x_next[0] = x[0] + x[1] * dt;
	x_next[1] = x[1];
}

static void cv_model_jacobian(const float *x, const float *u, float dt,
			      float *F, void *ctx)
{
	(void)x;
	(void)u;
	(void)ctx;
	F[0] = 1.0f;
	F[1] = dt;
	F[2] = 0.0f;
	F[3] = 1.0f;
}

static void cv_measure(const float *x, float *z, void *ctx)
{
	(void)ctx;
	z[0] = x[0];
}

static void cv_measure_jacobian(const float *x, float *H, void *ctx)
{
	(void)x;
	(void)ctx;
	H[0] = 1.0f;
	H[1] = 0.0f;
}

static void setup_cv(struct ekf_filter *filter, int analytic)
{
	const float x0[2] = { 0.0f, 0.0f };
	const float P0[4] = { 10.0f, 0.0f, 0.0f, 10.0f };
	const float Q[4] = { 1e-6f, 0.0f, 0.0f, 1e-6f };
	const float R[1] = { 0.25f };

	expect_status(ekf_filter_init(filter, 2, 1, x0, P0, Q, R),
		      EKF_FILTER_OK, "constant-velocity init");
	filter->f = cv_model;
	filter->h = cv_measure;
	filter->F = analytic ? cv_model_jacobian : NULL;
	filter->H = analytic ? cv_measure_jacobian : NULL;
}

/* ---- 1: everything a bad call must refuse ---------------------------- */

static void test_init_validates(void)
{
	struct ekf_filter filter;
	const float x0[2] = { 0.0f, 0.0f };
	const float P0[4] = { 1.0f, 0.0f, 0.0f, 1.0f };
	const float Q[4] = { 1e-3f, 0.0f, 0.0f, 1e-3f };
	const float R[1] = { 0.1f };
	float bad[4];

	expect_status(ekf_filter_init(NULL, 2, 1, x0, P0, Q, R),
		      EKF_FILTER_ERR_NULL, "init rejects a null filter");
	expect_status(ekf_filter_init(&filter, 2, 1, NULL, P0, Q, R),
		      EKF_FILTER_ERR_NULL, "init rejects a null x0");
	expect_status(ekf_filter_init(&filter, 2, 1, x0, NULL, Q, R),
		      EKF_FILTER_ERR_NULL, "init rejects a null P0");
	expect_status(ekf_filter_init(&filter, 2, 1, x0, P0, NULL, R),
		      EKF_FILTER_ERR_NULL, "init rejects a null Q");
	expect_status(ekf_filter_init(&filter, 2, 1, x0, P0, Q, NULL),
		      EKF_FILTER_ERR_NULL, "init rejects a null R");

	expect_status(ekf_filter_init(&filter, 0, 1, x0, P0, Q, R),
		      EKF_FILTER_ERR_INVALID_VALUE, "init rejects n = 0");
	expect_status(ekf_filter_init(&filter, 9, 1, x0, P0, Q, R),
		      EKF_FILTER_ERR_INVALID_VALUE, "init rejects n = 9");
	expect_status(ekf_filter_init(&filter, 2, 0, x0, P0, Q, R),
		      EKF_FILTER_ERR_INVALID_VALUE, "init rejects m = 0");
	expect_status(ekf_filter_init(&filter, 2, 5, x0, P0, Q, R),
		      EKF_FILTER_ERR_INVALID_VALUE, "init rejects m = 5");

	bad[0] = 1e-3f;
	bad[1] = 0.0f;
	bad[2] = 0.0f;
	bad[3] = (float)NAN;
	expect_status(ekf_filter_init(&filter, 2, 1, x0, P0, bad, R),
		      EKF_FILTER_ERR_NOT_FINITE, "init rejects a NaN in Q");
	bad[3] = (float)INFINITY;
	expect_status(ekf_filter_init(&filter, 2, 1, x0, P0, bad, R),
		      EKF_FILTER_ERR_NOT_FINITE, "init rejects an infinity in Q");

	bad[1] = 1e-3f;
	bad[2] = 2e-3f;
	bad[3] = 1e-3f;
	expect_status(ekf_filter_init(&filter, 2, 1, x0, P0, bad, R),
		      EKF_FILTER_ERR_INVALID_VALUE,
		      "init rejects a non-symmetric Q");

	bad[1] = 1e-3f;
	bad[2] = 1e-3f + 5e-7f;
	expect_status(ekf_filter_init(&filter, 2, 1, x0, P0, bad, R),
		      EKF_FILTER_OK, "init accepts symmetry within 1e-6");

	bad[0] = 1.0f;
	bad[1] = 0.5f;
	bad[2] = -0.5f;
	bad[3] = 1.0f;
	expect_status(ekf_filter_init(&filter, 2, 1, x0, bad, Q, R),
		      EKF_FILTER_ERR_INVALID_VALUE,
		      "init rejects a non-symmetric P0");

	expect_status(ekf_filter_init(&filter, 2, 1, x0, P0, Q, x0),
		      EKF_FILTER_OK, "init accepts a 1 x 1 R");
}

static void test_calls_before_init(void)
{
	struct ekf_filter filter;
	struct ekf_filter ready;
	const float x0[2] = { 0.0f, 0.0f };
	const float P0[4] = { 1.0f, 0.0f, 0.0f, 1.0f };
	const float z[1] = { 1.0f };

	memset(&filter, 0, sizeof(filter));

	expect_status(ekf_filter_predict(&filter, NULL, DT),
		      EKF_FILTER_ERR_NOT_INITIALIZED,
		      "predict refuses an uninitialised filter");
	expect_status(ekf_filter_update(&filter, z, NULL),
		      EKF_FILTER_ERR_NOT_INITIALIZED,
		      "update refuses an uninitialised filter");
	expect_status(ekf_filter_reset(&filter, x0, P0),
		      EKF_FILTER_ERR_NOT_INITIALIZED,
		      "reset refuses an uninitialised filter");

	expect_status(ekf_filter_predict(NULL, NULL, DT), EKF_FILTER_ERR_NULL,
		      "predict rejects a null filter");
	expect_status(ekf_filter_update(NULL, z, NULL), EKF_FILTER_ERR_NULL,
		      "update rejects a null filter");
	expect_status(ekf_filter_reset(NULL, x0, P0), EKF_FILTER_ERR_NULL,
		      "reset rejects a null filter");

	/* Initialised, but the model was never attached. */
	setup_cv(&ready, 1);
	ready.f = NULL;
	expect_status(ekf_filter_predict(&ready, NULL, DT),
		      EKF_FILTER_ERR_INVALID_VALUE, "predict needs f");
	expect_status(ekf_filter_update(&ready, z, NULL),
		      EKF_FILTER_ERR_INVALID_VALUE, "update needs f");
	setup_cv(&ready, 1);
	ready.h = NULL;
	expect_status(ekf_filter_update(&ready, z, NULL),
		      EKF_FILTER_ERR_INVALID_VALUE, "update needs h");

	setup_cv(&ready, 1);
	expect_status(ekf_filter_predict(&ready, NULL, 0.0f),
		      EKF_FILTER_ERR_INVALID_VALUE, "predict rejects dt = 0");
	expect_status(ekf_filter_predict(&ready, NULL, -DT),
		      EKF_FILTER_ERR_INVALID_VALUE,
		      "predict rejects a negative dt");
	expect_status(ekf_filter_predict(&ready, NULL, (float)NAN),
		      EKF_FILTER_ERR_NOT_FINITE, "predict rejects a NaN dt");
	expect_status(ekf_filter_update(&ready, NULL, NULL),
		      EKF_FILTER_ERR_NULL, "update rejects a null z");
	expect_status(ekf_filter_reset(&ready, NULL, P0), EKF_FILTER_ERR_NULL,
		      "reset rejects a null x0");
}

/* ---- 2: a linear tracker with analytic Jacobians ---------------------- */

static void test_constant_velocity(void)
{
	struct ekf_filter filter;
	unsigned long seed = 12345UL;
	float truth = 0.0f;
	float z[1];
	int step;

	setup_cv(&filter, 1);

	for (step = 0; step < 500; step++) {
		truth += TRUE_VELOCITY * DT;
		z[0] = truth + MEAS_NOISE * jitter(&seed);
		expect_status(ekf_filter_predict(&filter, NULL, DT),
			      EKF_FILTER_OK, "constant-velocity predict");
		expect_status(ekf_filter_update(&filter, z, NULL),
			      EKF_FILTER_OK, "constant-velocity update");
	}

	expect_within((double)filter.x[0], (double)truth, (double)MEAS_NOISE,
		      "position beats the measurement noise");
	expect_within((double)filter.x[1], (double)TRUE_VELOCITY,
		      0.05 * (double)TRUE_VELOCITY,
		      "velocity is within 5 percent");
}

/* ---- 3: the same run with the Jacobians differentiated numerically ---- */

static void test_numeric_jacobians_agree(void)
{
	struct ekf_filter analytic;
	struct ekf_filter numeric;
	unsigned long seed = 12345UL;
	float truth = 0.0f;
	float z[1];
	int step;

	setup_cv(&analytic, 1);
	setup_cv(&numeric, 0);

	for (step = 0; step < 500; step++) {
		truth += TRUE_VELOCITY * DT;
		z[0] = truth + MEAS_NOISE * jitter(&seed);

		ekf_filter_predict(&analytic, NULL, DT);
		ekf_filter_predict(&numeric, NULL, DT);
		ekf_filter_update(&analytic, z, NULL);
		expect_status(ekf_filter_update(&numeric, z, NULL),
			      EKF_FILTER_OK, "numeric-Jacobian update");

		expect_within((double)numeric.x[0], (double)analytic.x[0], 1e-3,
			      "numeric position matches the analytic one");
		expect_within((double)numeric.x[1], (double)analytic.x[1], 1e-3,
			      "numeric velocity matches the analytic one");
	}
}

/* ---- 4: a nonlinear sensor, range and bearing to a fixed target ------- */

struct observer {
	double x;
	double y;
};

static void target_model(const float *x, const float *u, float dt,
			 float *x_next, void *ctx)
{
	(void)u;
	(void)dt;
	(void)ctx;
	x_next[0] = x[0];
	x_next[1] = x[1];
}

static void range_bearing(const float *x, float *z, void *ctx)
{
	const struct observer *eye = (const struct observer *)ctx;
	double dx = (double)x[0] - eye->x;
	double dy = (double)x[1] - eye->y;

	z[0] = (float)sqrt(dx * dx + dy * dy);
	z[1] = (float)atan2(dy, dx);
}

static void test_nonlinear_measurement(void)
{
	struct ekf_filter filter;
	struct observer eye;
	const float x0[2] = { 5.6f, 3.8f };	/* 1.0 away from the truth */
	const float P0[4] = { 2.0f, 0.0f, 0.0f, 2.0f };
	const float Q[4] = { 1e-6f, 0.0f, 0.0f, 1e-6f };
	const float R[4] = { 4e-4f, 0.0f, 0.0f, 1.6e-5f };
	const float truth[2] = { 5.0f, 3.0f };
	unsigned long seed = 7777UL;
	float z[2];
	double error;
	int step;

	eye.x = 0.0;
	eye.y = 0.0;

	expect_status(ekf_filter_init(&filter, 2, 2, x0, P0, Q, R),
		      EKF_FILTER_OK, "range-bearing init");
	filter.f = target_model;
	filter.h = range_bearing;
	filter.ctx = &eye;			/* no Jacobians: numeric path */

	for (step = 0; step < 400; step++) {
		eye.x = 0.02 * (double)step;	/* the observer drifts along x */

		expect_status(ekf_filter_predict(&filter, NULL, DT),
			      EKF_FILTER_OK, "range-bearing predict");

		range_bearing(truth, z, &eye);
		z[0] += 0.02f * jitter(&seed);
		z[1] += 0.004f * jitter(&seed);

		expect_status(ekf_filter_update(&filter, z, NULL),
			      EKF_FILTER_OK, "range-bearing update");
	}

	error = sqrt(((double)filter.x[0] - 5.0) * ((double)filter.x[0] - 5.0) +
		     ((double)filter.x[1] - 3.0) * ((double)filter.x[1] - 3.0));
	expect_within(error, 0.0, 0.05, "the target is found from 1.0 away");
}

/* ---- 5: a long run must not tilt or deflate the covariance ----------- */

static void test_covariance_stays_sane(void)
{
	struct ekf_filter filter;
	unsigned long seed = 99UL;
	float truth = 0.0f;
	float z[1];
	int step;

	setup_cv(&filter, 1);

	for (step = 0; step < 10000; step++) {
		truth += TRUE_VELOCITY * DT;
		z[0] = truth + MEAS_NOISE * jitter(&seed);
		ekf_filter_predict(&filter, NULL, DT);
		ekf_filter_update(&filter, z, NULL);
	}

	expect_within((double)filter.P[1], (double)filter.P[2], 1e-6,
		      "P is still symmetric after 10000 cycles");
	expect_true(filter.P[0] > 0.0f, "P[0][0] is still positive");
	expect_true(filter.P[3] > 0.0f, "P[1][1] is still positive");
	expect_true(isfinite(filter.x[0]) && isfinite(filter.x[1]),
		    "the estimate is still finite");
}

/* ---- 6: a rejected update leaves the filter exactly as it was --------- */

static void scalar_model(const float *x, const float *u, float dt,
			 float *x_next, void *ctx)
{
	(void)u;
	(void)dt;
	(void)ctx;
	x_next[0] = x[0];
}

static void scalar_measure(const float *x, float *z, void *ctx)
{
	(void)ctx;
	z[0] = x[0];
}

static void test_singular_and_bad_input(void)
{
	struct ekf_filter filter;
	const float x0[1] = { 2.0f };
	const float zero[1] = { 0.0f };
	const float z[1] = { 1.0f };
	float bad[1];

	/* No uncertainty anywhere: S = H P H' + R is zero and cannot invert. */
	expect_status(ekf_filter_init(&filter, 1, 1, x0, zero, zero, zero),
		      EKF_FILTER_OK, "degenerate init");
	filter.f = scalar_model;
	filter.h = scalar_measure;

	expect_status(ekf_filter_update(&filter, z, NULL),
		      EKF_FILTER_ERR_NUMERIC,
		      "a singular innovation covariance is refused");
	expect_within((double)filter.x[0], 2.0, 0.0,
		      "the refused update left x alone");
	expect_within((double)filter.P[0], 0.0, 0.0,
		      "the refused update left P alone");

	setup_cv(&filter, 1);
	ekf_filter_predict(&filter, NULL, DT);

	bad[0] = (float)NAN;
	expect_status(ekf_filter_update(&filter, bad, NULL),
		      EKF_FILTER_ERR_NOT_FINITE, "a NaN measurement is refused");
	bad[0] = (float)INFINITY;
	expect_status(ekf_filter_update(&filter, bad, NULL),
		      EKF_FILTER_ERR_NOT_FINITE,
		      "an infinite measurement is refused");
	expect_within((double)filter.x[0], 0.0, 0.0,
		      "the refused measurement left x alone");
}

/* ---- 7: the innovation handed back is z - h(x) ------------------------ */

static void test_innovation_output(void)
{
	struct ekf_filter filter;
	float z[1];
	float innovation[1];
	float expected[1];
	int step;

	setup_cv(&filter, 1);

	for (step = 0; step < 20; step++) {
		z[0] = 3.0f + 0.1f * (float)step;
		ekf_filter_predict(&filter, NULL, DT);

		cv_measure(filter.x, expected, NULL);	/* h(x) before the update */
		expect_status(ekf_filter_update(&filter, z, innovation),
			      EKF_FILTER_OK, "update with an innovation out");
		expect_within((double)innovation[0],
			      (double)z[0] - (double)expected[0], 1e-6,
			      "innovation equals z - h(x)");
	}
}

/* ---- reset keeps the tuning and the model ---------------------------- */

static void test_reset(void)
{
	struct ekf_filter filter;
	const float x0[2] = { 42.0f, -1.0f };
	const float P0[4] = { 3.0f, 0.0f, 0.0f, 3.0f };
	float z[1];
	int step;

	setup_cv(&filter, 1);
	for (step = 0; step < 50; step++) {
		z[0] = (float)step;
		ekf_filter_predict(&filter, NULL, DT);
		ekf_filter_update(&filter, z, NULL);
	}

	expect_status(ekf_filter_reset(&filter, x0, P0), EKF_FILTER_OK,
		      "reset succeeds");
	expect_within((double)filter.x[0], 42.0, 0.0, "reset sets x");
	expect_within((double)filter.x[1], -1.0, 0.0, "reset sets the rest of x");
	expect_within((double)filter.P[0], 3.0, 0.0, "reset sets P");
	expect_within((double)filter.Q[0], (double)1e-6f, 0.0,
		      "reset leaves Q alone");
	expect_within((double)filter.R[0], 0.25, 0.0, "reset leaves R alone");
	expect_true(filter.f == cv_model && filter.h == cv_measure,
		    "reset leaves the model alone");
}

/* ---- 8: the legacy update is the update it was ----------------------- *
 *
 * Captured from the library before the sensor, gate, wrap and Q-rate work,
 * with every new field at its init default. The relative tolerance leaves
 * room for a compiler that contracts to FMA; on the reference host the
 * values come out bit for bit. */

static const double golden_cv[6] = {
	0x1.905cbcp+5, 0x1.00e76ep+0,
	0x1.462018p-8, 0x1.039bbap-11, 0x1.039bbap-11, 0x1.a59eb6p-14,
};

static const double golden_range_bearing[6] = {
	0x1.40217ep+2, 0x1.807f38p+1,
	0x1.27781ep-16, -0x1.d3ef58p-20, -0x1.d3ef58p-20, 0x1.2c20eap-16,
};

static void expect_golden(const struct ekf_filter *filter,
			  const double *golden, const char *name)
{
	int i;

	for (i = 0; i < 2; i++)
		expect_within((double)filter->x[i], golden[i],
			      1e-5 * fabs(golden[i]), name);
	for (i = 0; i < 4; i++)
		expect_within((double)filter->P[i], golden[2 + i],
			      1e-5 * fabs(golden[2 + i]), name);
}

static void test_legacy_regression(void)
{
	struct ekf_filter filter;
	struct observer eye;
	const float x0[2] = { 5.6f, 3.8f };
	const float P0[4] = { 2.0f, 0.0f, 0.0f, 2.0f };
	const float Q[4] = { 1e-6f, 0.0f, 0.0f, 1e-6f };
	const float R[4] = { 4e-4f, 0.0f, 0.0f, 1.6e-5f };
	const float truth[2] = { 5.0f, 3.0f };
	unsigned long seed = 12345UL;
	float position = 0.0f;
	float z[2];
	int step;

	setup_cv(&filter, 1);
	for (step = 0; step < 500; step++) {
		position += TRUE_VELOCITY * DT;
		z[0] = position + MEAS_NOISE * jitter(&seed);
		ekf_filter_predict(&filter, NULL, DT);
		ekf_filter_update(&filter, z, NULL);
	}
	expect_golden(&filter, golden_cv, "constant velocity matches the old run");

	ekf_filter_init(&filter, 2, 2, x0, P0, Q, R);
	filter.f = target_model;
	filter.h = range_bearing;
	filter.ctx = &eye;
	eye.y = 0.0;
	seed = 7777UL;
	for (step = 0; step < 400; step++) {
		eye.x = 0.02 * (double)step;
		ekf_filter_predict(&filter, NULL, DT);
		range_bearing(truth, z, &eye);
		z[0] += 0.02f * jitter(&seed);
		z[1] += 0.004f * jitter(&seed);
		ekf_filter_update(&filter, z, NULL);
	}
	expect_golden(&filter, golden_range_bearing,
		      "range-bearing matches the old run");

}

int main(void)
{
	test_init_validates();
	test_calls_before_init();
	test_constant_velocity();
	test_numeric_jacobians_agree();
	test_nonlinear_measurement();
	test_covariance_stays_sane();
	test_singular_and_bad_input();
	test_innovation_output();
	test_reset();
	test_legacy_regression();

	printf("PASS: %d checks\n", tests_run);
	return EXIT_SUCCESS;
}
