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
	const float zero_state = 0.0f;
	const float unit = 1.0f;
	const float huge = 1e20f;
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

	/* A reading so far out its NIS overflows a float. The old update took
	 * it, and with no gate it still must: x halfway, P halved. */
	ekf_filter_init(&filter, 1, 1, &zero_state, &unit, &zero_state, &unit);
	filter.f = scalar_model;
	filter.h = scalar_measure;
	expect_status(ekf_filter_update(&filter, &huge, NULL), EKF_FILTER_OK,
		      "an ungated update takes a NIS past FLT_MAX");
	expect_within((double)filter.x[0], 5e19, 1e14, "and moves x halfway");
	expect_within((double)filter.P[0], 0.5, 1e-7, "and halves P");
	expect_true(isinf(filter.nis) && filter.nis > 0.0f,
		    "and stores that NIS as infinity");

	filter.gate = 9.0f;
	expect_status(ekf_filter_update(&filter, &huge, NULL),
		      EKF_FILTER_REJECTED, "a gate rejects an infinite NIS");
}

/* ---- 9: the filter's own sensor, passed as a sensor, changes nothing -- */

static void cv_sensor(struct ekf_sensor *sensor)
{
	memset(sensor, 0, sizeof(*sensor));
	sensor->m = 1;
	sensor->h = cv_measure;
	sensor->H = cv_measure_jacobian;
	sensor->R[0] = 0.25f;
}

static void test_update_sensor_matches_update(void)
{
	struct ekf_filter plain;
	struct ekf_filter sensed;
	struct ekf_sensor sensor;
	unsigned long seed = 4242UL;
	float truth = 0.0f;
	float z[1];
	float a[1];
	float b[1];
	int step;

	setup_cv(&plain, 1);
	setup_cv(&sensed, 1);
	cv_sensor(&sensor);

	for (step = 0; step < 200; step++) {
		truth += TRUE_VELOCITY * DT;
		z[0] = truth + MEAS_NOISE * jitter(&seed);
		ekf_filter_predict(&plain, NULL, DT);
		ekf_filter_predict(&sensed, NULL, DT);
		expect_status(ekf_filter_update(&plain, z, a), EKF_FILTER_OK,
			      "own-sensor reference update");
		expect_status(ekf_filter_update_sensor(&sensed, &sensor, z, b),
			      EKF_FILTER_OK, "update_sensor with the same sensor");
	}

	expect_true(memcmp(plain.x, sensed.x, sizeof(plain.x)) == 0 &&
		    memcmp(plain.P, sensed.P, sizeof(plain.P)) == 0,
		    "update_sensor gives update's x and P bit for bit");
	expect_true(a[0] == b[0] && plain.nis == sensed.nis,
		    "and the same innovation and NIS");
}

/* ---- 10: two sensors of different size feed one filter --------------- */

static void pair_measure(const float *x, float *z, void *ctx)
{
	(void)ctx;
	z[0] = x[0];
	z[1] = x[1];
}

static void test_two_sensors(void)
{
	struct ekf_filter filter;
	struct ekf_sensor position;
	struct ekf_sensor both;
	unsigned long seed = 31337UL;
	float truth = 0.0f;
	float z[2];
	int step;

	setup_cv(&filter, 1);
	cv_sensor(&position);
	memset(&both, 0, sizeof(both));
	both.m = 2;
	both.h = pair_measure;			/* no H: numeric */
	both.R[0] = 1.0f;
	both.R[3] = 0.01f;

	for (step = 0; step < 300; step++) {
		truth += TRUE_VELOCITY * DT;
		ekf_filter_predict(&filter, NULL, DT);

		z[0] = truth + MEAS_NOISE * jitter(&seed);
		expect_status(ekf_filter_update_sensor(&filter, &position, z, NULL),
			      EKF_FILTER_OK, "the 1-value sensor updates");

		if (step % 5 == 0) {
			z[0] = truth + jitter(&seed);
			z[1] = TRUE_VELOCITY + 0.1f * jitter(&seed);
			expect_status(ekf_filter_update_sensor(&filter, &both, z,
							       NULL),
				      EKF_FILTER_OK, "the 2-value sensor updates");
		}
	}

	expect_within((double)filter.x[0], (double)truth, (double)MEAS_NOISE,
		      "fused position beats the noise");
	expect_within((double)filter.x[1], (double)TRUE_VELOCITY,
		      0.02 * (double)TRUE_VELOCITY,
		      "fused velocity is within 2 percent");
}

/* ---- 11: a bad sensor, or a bad gate, is refused ---------------------- */

static void test_sensor_validation(void)
{
	struct ekf_filter filter;
	struct ekf_filter blank;
	struct ekf_sensor sensor;
	const float z[2] = { 1.0f, 1.0f };

	setup_cv(&filter, 1);
	memset(&blank, 0, sizeof(blank));
	cv_sensor(&sensor);

	expect_status(ekf_filter_update_sensor(NULL, &sensor, z, NULL),
		      EKF_FILTER_ERR_NULL, "update_sensor rejects a null filter");
	expect_status(ekf_filter_update_sensor(&blank, &sensor, z, NULL),
		      EKF_FILTER_ERR_NOT_INITIALIZED,
		      "update_sensor refuses an uninitialised filter");
	expect_status(ekf_filter_update_sensor(&filter, NULL, z, NULL),
		      EKF_FILTER_ERR_NULL, "update_sensor rejects a null sensor");
	expect_status(ekf_filter_update_sensor(&filter, &sensor, NULL, NULL),
		      EKF_FILTER_ERR_NULL, "update_sensor rejects a null z");

	sensor.m = 0;
	expect_status(ekf_filter_update_sensor(&filter, &sensor, z, NULL),
		      EKF_FILTER_ERR_INVALID_VALUE, "sensor m = 0 is refused");
	sensor.m = 5;
	expect_status(ekf_filter_update_sensor(&filter, &sensor, z, NULL),
		      EKF_FILTER_ERR_INVALID_VALUE, "sensor m = 5 is refused");

	cv_sensor(&sensor);
	sensor.h = NULL;
	expect_status(ekf_filter_update_sensor(&filter, &sensor, z, NULL),
		      EKF_FILTER_ERR_INVALID_VALUE, "a sensor needs h");

	cv_sensor(&sensor);
	sensor.R[0] = (float)NAN;
	expect_status(ekf_filter_update_sensor(&filter, &sensor, z, NULL),
		      EKF_FILTER_ERR_NOT_FINITE, "a NaN in sensor R is refused");

	cv_sensor(&sensor);
	sensor.m = 2;
	sensor.h = pair_measure;
	sensor.H = NULL;
	sensor.R[0] = 1.0f;
	sensor.R[1] = 0.5f;
	sensor.R[2] = -0.5f;
	sensor.R[3] = 1.0f;
	expect_status(ekf_filter_update_sensor(&filter, &sensor, z, NULL),
		      EKF_FILTER_ERR_INVALID_VALUE,
		      "a non-symmetric sensor R is refused");

	cv_sensor(&sensor);
	sensor.gate = -1.0f;
	expect_status(ekf_filter_update_sensor(&filter, &sensor, z, NULL),
		      EKF_FILTER_ERR_INVALID_VALUE, "a negative gate is refused");
	sensor.gate = (float)NAN;
	expect_status(ekf_filter_update_sensor(&filter, &sensor, z, NULL),
		      EKF_FILTER_ERR_NOT_FINITE, "a NaN gate is refused");
	sensor.gate = (float)INFINITY;
	expect_status(ekf_filter_update_sensor(&filter, &sensor, z, NULL),
		      EKF_FILTER_ERR_NOT_FINITE, "an infinite gate is refused");

	cv_sensor(&sensor);
	filter.f = NULL;
	expect_status(ekf_filter_update_sensor(&filter, &sensor, z, NULL),
		      EKF_FILTER_ERR_INVALID_VALUE, "update_sensor needs f");

	setup_cv(&filter, 1);
	filter.gate = -1.0f;
	expect_status(ekf_filter_update(&filter, z, NULL),
		      EKF_FILTER_ERR_INVALID_VALUE,
		      "update refuses a negative filter gate");
	filter.gate = (float)NAN;
	expect_status(ekf_filter_update(&filter, z, NULL),
		      EKF_FILTER_ERR_NOT_FINITE,
		      "update refuses a NaN filter gate");

	expect_within((double)filter.x[0], 0.0, 0.0,
		      "the refused calls left x alone");
	expect_within((double)filter.P[0], 10.0, 0.0,
		      "the refused calls left P alone");
}

/* ---- 12: an outlier is gated out, and nothing but nis moves ----------- */

/* Counts its calls through ctx and leaves the state alone. */
static void count_normalize(float *x, void *ctx)
{
	(void)x;
	(*(int *)ctx)++;
}

static void test_gate_rejects(void)
{
	struct ekf_filter filter;
	struct ekf_filter before;
	struct ekf_sensor sensor;
	float z[1];
	float innovation[1];
	double nis;
	int calls = 0;
	int step;

	setup_cv(&filter, 1);
	for (step = 0; step < 100; step++) {
		z[0] = TRUE_VELOCITY * DT * (float)(step + 1);
		ekf_filter_predict(&filter, NULL, DT);
		ekf_filter_update(&filter, z, NULL);
	}

	filter.gate = 9.0f;
	filter.normalize = count_normalize;
	filter.ctx = &calls;			/* the CV callbacks ignore it */
	memcpy(&before, &filter, sizeof(filter));
	z[0] = filter.x[0] + 100.0f;		/* a spike */
	nis = 100.0 * 100.0 / ((double)filter.P[0] + 0.25);

	expect_status(ekf_filter_update(&filter, z, innovation),
		      EKF_FILTER_REJECTED, "a spike past the gate is rejected");
	before.nis = filter.nis;
	expect_true(memcmp(&filter, &before, sizeof(filter)) == 0,
		    "the rejected update changed nothing but nis");
	expect_true(calls == 0, "a rejected update does not normalize");
	expect_within((double)innovation[0], 100.0, 1e-4,
		      "the rejected update still wrote the innovation");
	expect_within((double)filter.nis, nis, 1e-6 * nis,
		      "the rejected update still wrote nis");

	z[0] = filter.x[0] + 0.1f;
	expect_status(ekf_filter_update(&filter, z, NULL), EKF_FILTER_OK,
		      "a measurement inside the gate is taken");

	expect_true(calls == 1, "an accepted update does normalize");

	cv_sensor(&sensor);
	sensor.gate = 9.0f;
	memcpy(&before, &filter, sizeof(filter));
	z[0] = filter.x[0] - 100.0f;
	expect_status(ekf_filter_update_sensor(&filter, &sensor, z, NULL),
		      EKF_FILTER_REJECTED, "a sensor gate rejects too");
	before.nis = filter.nis;
	expect_true(memcmp(&filter, &before, sizeof(filter)) == 0,
		    "and changes nothing but nis");
	expect_true(calls == 1, "and does not normalize");
}

/* ---- 13: NIS against a 2 x 2 worked by hand -------------------------- */

static void test_nis_by_hand(void)
{
	struct ekf_filter filter;
	const float x0[2] = { 0.0f, 0.0f };
	const float P0[4] = { 2.0f, 0.5f, 0.5f, 1.0f };
	const float Q[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	const float R[4] = { 0.5f, 0.1f, 0.1f, 0.25f };
	const float z[2] = { 1.0f, -0.5f };
	double s00, s01, s11, det, nis;

	expect_status(ekf_filter_init(&filter, 2, 2, x0, P0, Q, R),
		      EKF_FILTER_OK, "NIS init");
	filter.f = target_model;
	filter.h = pair_measure;
	expect_within((double)filter.nis, 0.0, 0.0, "init clears nis");

	/* S = P + R, and y' S^-1 y through the 2 x 2 adjugate. */
	s00 = (double)P0[0] + (double)R[0];
	s01 = (double)P0[1] + (double)R[1];
	s11 = (double)P0[3] + (double)R[3];
	det = s00 * s11 - s01 * s01;
	nis = (s11 * (double)z[0] * (double)z[0] -
	       2.0 * s01 * (double)z[0] * (double)z[1] +
	       s00 * (double)z[1] * (double)z[1]) / det;

	expect_status(ekf_filter_update(&filter, z, NULL), EKF_FILTER_OK,
		      "NIS update");
	expect_within((double)filter.nis, nis, 1e-6, "NIS matches y' S^-1 y");

	expect_status(ekf_filter_reset(&filter, x0, P0), EKF_FILTER_OK,
		      "NIS reset");
	expect_within((double)filter.nis, 0.0, 0.0, "reset clears nis");
}

/* ---- 14: a heading measured across +/- pi does not jump 2 pi ---------- */

#define PI 3.14159265358979323846

static double wrap(double angle)
{
	while (angle > PI)
		angle -= 2.0 * PI;
	while (angle <= -PI)
		angle += 2.0 * PI;

	return angle;
}

static void wrapped_residual(const float *z, const float *predicted, float *y,
			     void *ctx)
{
	(void)ctx;
	y[0] = (float)wrap((double)z[0] - (double)predicted[0]);
}

static void wrap_heading(float *x, void *ctx)
{
	int *calls = (int *)ctx;

	if (calls)
		(*calls)++;
	x[0] = (float)wrap((double)x[0]);
}

static void test_residual_wrap(void)
{
	struct ekf_filter filter;
	const float x0[1] = { 3.0f };
	const float P0[1] = { 0.1f };
	const float Q[1] = { 1e-6f };
	const float R[1] = { 0.01f };
	const double truth = PI - 0.01;
	unsigned long seed = 555UL;
	float z[1];
	float last;
	int crossed = 0;
	int step;

	expect_status(ekf_filter_init(&filter, 1, 1, x0, P0, Q, R),
		      EKF_FILTER_OK, "heading init");
	filter.f = scalar_model;
	filter.h = scalar_measure;
	filter.residual = wrapped_residual;
	filter.normalize = wrap_heading;

	for (step = 0; step < 200; step++) {
		z[0] = (float)wrap(truth + 0.05 * (double)jitter(&seed));
		if (z[0] < 0.0f)
			crossed++;

		last = filter.x[0];
		ekf_filter_predict(&filter, NULL, DT);
		expect_status(ekf_filter_update(&filter, z, NULL), EKF_FILTER_OK,
			      "heading update");
		expect_true(fabs(wrap((double)filter.x[0] - (double)last)) < 0.2,
			    "the heading never jumps");
		expect_true(filter.x[0] > -PI && filter.x[0] <= PI,
			    "the heading stays in (-pi, pi]");
	}

	expect_true(crossed > 20, "the measurements did cross -pi");
	expect_within(wrap((double)filter.x[0] - truth), 0.0, 0.02,
		      "the heading settles on the truth near +pi");
}

/* ---- 15: normalize runs after predict and after update ---------------- */

static void spin(const float *x, const float *u, float dt, float *x_next,
		 void *ctx)
{
	(void)u;
	(void)ctx;
	x_next[0] = x[0] + 1.0f * dt;
}

static void test_normalize_applied(void)
{
	struct ekf_filter filter;
	const float x0[1] = { 3.1f };
	const float P0[1] = { 0.1f };
	const float Q[1] = { 1e-6f };
	const float R[1] = { 0.01f };
	const float z[1] = { -3.0f };
	int calls = 0;

	ekf_filter_init(&filter, 1, 1, x0, P0, Q, R);
	filter.f = spin;
	filter.h = scalar_measure;
	filter.residual = wrapped_residual;
	filter.normalize = wrap_heading;
	filter.ctx = &calls;

	expect_status(ekf_filter_predict(&filter, NULL, DT), EKF_FILTER_OK,
		      "spin predict");
	expect_within((double)filter.x[0], (double)(3.1f + 0.1f) - 2.0 * PI,
		      1e-5, "predict wrapped 3.2 to 3.2 - 2 pi");
	expect_true(calls == 1, "predict called normalize once, with ctx");

	filter.x[0] = 3.13f;			/* updates towards -3.0 + 2 pi */
	expect_status(ekf_filter_update(&filter, z, NULL), EKF_FILTER_OK,
		      "spin update");
	expect_true(calls == 2, "update called normalize once more");
	expect_true(filter.x[0] > -PI && filter.x[0] <= PI,
		    "the updated heading was wrapped");
	expect_within(wrap((double)filter.x[0] - (-3.0)), 0.0, 0.1,
		      "and moved towards the measurement");
}

/* ---- 16: q_per_second scales Q by dt --------------------------------- */

static void test_q_per_second(void)
{
	struct ekf_filter rate;
	struct ekf_filter step;
	const float x0[2] = { 0.0f, 1.0f };
	const float P0[4] = { 2.0f, 0.5f, 0.5f, 1.0f };
	const float Q[4] = { 0.5f, 0.1f, 0.1f, 0.2f };
	const float R[1] = { 0.25f };
	const float dt = 0.25f;
	double fpf[4];
	int i;

	ekf_filter_init(&rate, 2, 1, x0, P0, Q, R);
	rate.f = cv_model;
	rate.F = cv_model_jacobian;
	rate.h = cv_measure;
	step = rate;
	rate.q_per_second = 1;

	/* F = [1 dt; 0 1], so F P F' by hand. */
	fpf[0] = 2.0 + 2.0 * (double)dt * 0.5 + (double)dt * (double)dt * 1.0;
	fpf[1] = 0.5 + (double)dt * 1.0;
	fpf[2] = fpf[1];
	fpf[3] = 1.0;

	expect_status(ekf_filter_predict(&rate, NULL, dt), EKF_FILTER_OK,
		      "Q-rate predict");
	expect_status(ekf_filter_predict(&step, NULL, dt), EKF_FILTER_OK,
		      "Q-step predict");
	for (i = 0; i < 4; i++) {
		expect_within((double)rate.P[i],
			      fpf[i] + (double)Q[i] * (double)dt, 1e-6,
			      "q_per_second gives F P F' + Q dt");
		expect_within((double)step.P[i], fpf[i] + (double)Q[i], 1e-6,
			      "the default gives F P F' + Q");
	}
}

/* ---- 17: a callback that hands back NaN is refused, cleanly ---------- */

static void nan_residual(const float *z, const float *predicted, float *y,
			 void *ctx)
{
	(void)z;
	(void)predicted;
	(void)ctx;
	y[0] = (float)NAN;
}

static void nan_normalize(float *x, void *ctx)
{
	(void)ctx;
	x[0] = (float)NAN;
}

static void nan_model(const float *x, const float *u, float dt, float *x_next,
		      void *ctx)
{
	(void)x;
	(void)u;
	(void)dt;
	(void)ctx;
	x_next[0] = (float)NAN;
	x_next[1] = 0.0f;
}

static void test_callbacks_not_finite(void)
{
	struct ekf_filter filter;
	struct ekf_filter before;
	const float z[1] = { 1.0f };
	int calls = 0;

	setup_cv(&filter, 1);
	ekf_filter_predict(&filter, NULL, DT);
	ekf_filter_update(&filter, z, NULL);	/* a nonzero nis to keep */

	filter.residual = nan_residual;
	memcpy(&before, &filter, sizeof(filter));
	expect_status(ekf_filter_update(&filter, z, NULL),
		      EKF_FILTER_ERR_NOT_FINITE, "a NaN residual is refused");
	expect_true(memcmp(&filter, &before, sizeof(filter)) == 0,
		    "and changes nothing, nis included");

	filter.residual = NULL;
	filter.normalize = nan_normalize;
	memcpy(&before, &filter, sizeof(filter));
	expect_status(ekf_filter_update(&filter, z, NULL),
		      EKF_FILTER_ERR_NOT_FINITE,
		      "a NaN from normalize after update is refused");
	expect_true(memcmp(&filter, &before, sizeof(filter)) == 0,
		    "and changes nothing, nis included");
	expect_status(ekf_filter_predict(&filter, NULL, DT),
		      EKF_FILTER_ERR_NOT_FINITE,
		      "a NaN from normalize after predict is refused");
	expect_true(memcmp(&filter, &before, sizeof(filter)) == 0,
		    "and changes nothing");

	filter.f = nan_model;
	filter.F = cv_model_jacobian;
	filter.normalize = count_normalize;
	filter.ctx = &calls;
	memcpy(&before, &filter, sizeof(filter));
	expect_status(ekf_filter_predict(&filter, NULL, DT),
		      EKF_FILTER_ERR_NOT_FINITE, "a NaN from f is refused");
	expect_true(calls == 0, "before normalize ever sees it");
	expect_true(memcmp(&filter, &before, sizeof(filter)) == 0,
		    "and changes nothing");
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
	test_update_sensor_matches_update();
	test_two_sensors();
	test_sensor_validation();
	test_gate_rejects();
	test_nis_by_hand();
	test_residual_wrap();
	test_normalize_applied();
	test_q_per_second();
	test_callbacks_not_finite();

	printf("PASS: %d checks\n", tests_run);
	return EXIT_SUCCESS;
}
