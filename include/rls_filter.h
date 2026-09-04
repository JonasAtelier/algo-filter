/*
 * rls_filter.h - recursive least squares: fit y = phi . theta one sample at a
 * time.
 *
 * You have a log of inputs and outputs and a model that is linear in the
 * numbers you want. Write each sample as an output y and a regressor phi -- the
 * measured things y is made of -- and this finds the theta that best explains
 * them, updating on every sample instead of solving over a batch.
 *
 * C99, libm only. No allocation, no globals, no I/O. All state lives in a
 * struct you own; separate instances are independent.
 */
#ifndef RLS_FILTER_H
#define RLS_FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RLS_FILTER_MAX_PARAMS 8

enum rls_filter_status {
	RLS_FILTER_OK = 0,
	RLS_FILTER_ERR_NULL = -1,
	RLS_FILTER_ERR_NOT_FINITE = -2,
	RLS_FILTER_ERR_INVALID_VALUE = -3,
	RLS_FILTER_ERR_NOT_INITIALIZED = -4,
	RLS_FILTER_ERR_NUMERIC = -5,
};

struct rls_filter {
	int   n;                                                  /* parameters, 1..RLS_FILTER_MAX_PARAMS */
	float theta[RLS_FILTER_MAX_PARAMS];                       /* the estimate: read it */
	float P[RLS_FILTER_MAX_PARAMS * RLS_FILTER_MAX_PARAMS];   /* its covariance, n x n row-major */
	float lambda;                                             /* forgetting factor, (0, 1]; 1 = remember everything */
	uint32_t marker;
};

/* theta = 0, P = p0 * I. p0 large (1e3..1e6) means "I know nothing yet". lambda in (0, 1]. */
enum rls_filter_status rls_filter_init(struct rls_filter *filter, int n, float lambda, float p0);

/* One sample: the regressor phi (n values) and the output y it produced. error, if not
 * NULL, receives y - phi . theta computed before the update (the prediction error). */
enum rls_filter_status rls_filter_update(struct rls_filter *filter, const float *phi, float y, float *error);

/* What the current estimate predicts for a regressor. NAN if the filter is invalid. */
float rls_filter_predict(const struct rls_filter *filter, const float *phi);

/* Back to theta = 0, P = p0 * I, keeping n and lambda. */
enum rls_filter_status rls_filter_reset(struct rls_filter *filter, float p0);

#ifdef __cplusplus
}
#endif

#endif
