/*
 * Recursive least squares with exponential forgetting.
 *
 * Per sample, with P the covariance of the estimate and lambda the forgetting
 * factor:
 *
 *   e     = y - phi' theta            the error before the update
 *   denom = lambda + phi' P phi       how much this sample tells us
 *   K     = P phi / denom             the gain
 *   theta = theta + K e
 *   P     = (P - K phi' P) / lambda   then symmetrised
 *
 * The state is float so it is cheap to keep and easy to read, but every step
 * above runs in double and is checked before anything is written back: a call
 * that returns an error leaves the filter exactly as it found it.
 */
#include "rls_filter.h"

#include <float.h>
#include <math.h>

#define RLS_FILTER_MARKER UINT32_C(0x524c5346)

static int rls_filter_all_finite(const float *v, int count)
{
	int i;

	for (i = 0; i < count; i++) {
		if (!isfinite(v[i]))
			return 0;
	}

	return 1;
}

/*
 * The fields are public, so a caller can leave the struct in a state the maths
 * cannot survive. Every entry point checks this first.
 */
static int rls_filter_is_valid(const struct rls_filter *filter)
{
	if (filter->marker != RLS_FILTER_MARKER)
		return 0;

	if (filter->n < 1 || filter->n > RLS_FILTER_MAX_PARAMS)
		return 0;

	if (!isfinite(filter->lambda) || filter->lambda <= 0.0f ||
	    filter->lambda > 1.0f)
		return 0;

	if (!rls_filter_all_finite(filter->theta, filter->n))
		return 0;

	return rls_filter_all_finite(filter->P, filter->n * filter->n);
}

static enum rls_filter_status rls_filter_check_p0(float p0)
{
	if (!isfinite(p0))
		return RLS_FILTER_ERR_NOT_FINITE;

	if (p0 <= 0.0f)
		return RLS_FILTER_ERR_INVALID_VALUE;

	return RLS_FILTER_OK;
}

/* theta = 0, P = p0 * I over the live n x n block. n must already be set. */
static void rls_filter_seed(struct rls_filter *filter, float p0)
{
	int i;

	for (i = 0; i < RLS_FILTER_MAX_PARAMS; i++)
		filter->theta[i] = 0.0f;

	for (i = 0; i < RLS_FILTER_MAX_PARAMS * RLS_FILTER_MAX_PARAMS; i++)
		filter->P[i] = 0.0f;

	for (i = 0; i < filter->n; i++)
		filter->P[i * filter->n + i] = p0;
}

/* A double that must survive the cast back to float. */
static int rls_filter_storable(double x)
{
	return isfinite(x) && fabs(x) <= (double)FLT_MAX;
}

enum rls_filter_status
rls_filter_init(struct rls_filter *filter, int n, float lambda, float p0)
{
	enum rls_filter_status status;

	if (!filter)
		return RLS_FILTER_ERR_NULL;

	if (!isfinite(lambda))
		return RLS_FILTER_ERR_NOT_FINITE;

	status = rls_filter_check_p0(p0);
	if (status != RLS_FILTER_OK)
		return status;

	if (n < 1 || n > RLS_FILTER_MAX_PARAMS ||
	    lambda <= 0.0f || lambda > 1.0f)
		return RLS_FILTER_ERR_INVALID_VALUE;

	filter->n = n;
	filter->lambda = lambda;
	filter->marker = RLS_FILTER_MARKER;
	rls_filter_seed(filter, p0);

	return RLS_FILTER_OK;
}

enum rls_filter_status
rls_filter_update(struct rls_filter *filter, const float *phi, float y,
		  float *error)
{
	double p_phi[RLS_FILTER_MAX_PARAMS];
	double gain[RLS_FILTER_MAX_PARAMS];
	double theta[RLS_FILTER_MAX_PARAMS];
	double next_p[RLS_FILTER_MAX_PARAMS * RLS_FILTER_MAX_PARAMS];
	double lambda;
	double denom;
	double e;
	int n;
	int i;
	int j;

	if (!filter || !phi)
		return RLS_FILTER_ERR_NULL;

	if (!rls_filter_is_valid(filter))
		return RLS_FILTER_ERR_NOT_INITIALIZED;

	n = filter->n;

	if (!isfinite(y) || !rls_filter_all_finite(phi, n))
		return RLS_FILTER_ERR_NOT_FINITE;

	lambda = (double)filter->lambda;

	e = (double)y;
	for (i = 0; i < n; i++)
		e -= (double)phi[i] * (double)filter->theta[i];

	/* p_phi = P phi, and denom = lambda + phi' P phi alongside it. */
	denom = lambda;
	for (i = 0; i < n; i++) {
		p_phi[i] = 0.0;
		for (j = 0; j < n; j++)
			p_phi[i] += (double)filter->P[i * n + j] *
				    (double)phi[j];

		denom += (double)phi[i] * p_phi[i];
	}

	/*
	 * denom is lambda plus a quadratic form in a covariance that should be
	 * positive definite, so it can only reach zero if P has lost that
	 * property to rounding. Refuse the sample rather than divide by it.
	 */
	if (!isfinite(denom) || denom <= 0.0)
		return RLS_FILTER_ERR_NUMERIC;

	for (i = 0; i < n; i++) {
		gain[i] = p_phi[i] / denom;
		theta[i] = (double)filter->theta[i] + gain[i] * e;
	}

	/*
	 * P = (P - K phi' P) / lambda. P is symmetric, so phi' P is p_phi
	 * transposed. Averaging the two triangles costs nothing and stops
	 * rounding from letting them drift apart over a long run.
	 */
	for (i = 0; i < n; i++) {
		for (j = 0; j <= i; j++) {
			double upper;
			double lower;
			double mean;

			lower = ((double)filter->P[i * n + j] -
				 gain[i] * p_phi[j]) / lambda;
			upper = ((double)filter->P[j * n + i] -
				 gain[j] * p_phi[i]) / lambda;
			mean = 0.5 * (lower + upper);

			next_p[i * n + j] = mean;
			next_p[j * n + i] = mean;
		}
	}

	if (!rls_filter_storable(e))
		return RLS_FILTER_ERR_NUMERIC;

	for (i = 0; i < n; i++) {
		if (!rls_filter_storable(theta[i]))
			return RLS_FILTER_ERR_NUMERIC;
	}

	for (i = 0; i < n * n; i++) {
		if (!rls_filter_storable(next_p[i]))
			return RLS_FILTER_ERR_NUMERIC;
	}

	/*
	 * Nothing below can fail, so this is the commit. error is written
	 * first on purpose: should it point into theta or P, the writes that
	 * follow overwrite it, which is cheaper than detecting the aliasing.
	 */
	if (error)
		*error = (float)e;

	for (i = 0; i < n; i++)
		filter->theta[i] = (float)theta[i];

	for (i = 0; i < n * n; i++)
		filter->P[i] = (float)next_p[i];

	return RLS_FILTER_OK;
}

float rls_filter_predict(const struct rls_filter *filter, const float *phi)
{
	double sum;
	int i;

	if (!filter || !phi)
		return (float)NAN;

	if (!rls_filter_is_valid(filter))
		return (float)NAN;

	if (!rls_filter_all_finite(phi, filter->n))
		return (float)NAN;

	sum = 0.0;
	for (i = 0; i < filter->n; i++)
		sum += (double)phi[i] * (double)filter->theta[i];

	if (!rls_filter_storable(sum))
		return (float)NAN;

	return (float)sum;
}

enum rls_filter_status rls_filter_reset(struct rls_filter *filter, float p0)
{
	enum rls_filter_status status;

	if (!filter)
		return RLS_FILTER_ERR_NULL;

	if (!rls_filter_is_valid(filter))
		return RLS_FILTER_ERR_NOT_INITIALIZED;

	status = rls_filter_check_p0(p0);
	if (status != RLS_FILTER_OK)
		return status;

	rls_filter_seed(filter, p0);

	return RLS_FILTER_OK;
}
