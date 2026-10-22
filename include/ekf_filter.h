/* ekf_filter.h - extended Kalman filter for small state vectors. */
#ifndef EKF_FILTER_H
#define EKF_FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EKF_FILTER_MAX_STATES 8
#define EKF_FILTER_MAX_MEAS   4

enum ekf_filter_status {
	EKF_FILTER_OK = 0,
	EKF_FILTER_ERR_NULL = -1,
	EKF_FILTER_ERR_NOT_FINITE = -2,
	EKF_FILTER_ERR_INVALID_VALUE = -3,   /* bad size, missing model, non-symmetric matrix */
	EKF_FILTER_ERR_NOT_INITIALIZED = -4, /* marker not set: init was never called */
	EKF_FILTER_ERR_NUMERIC = -5,         /* innovation covariance not invertible */
};

/* Your plant: x_next = f(x, u, dt). u may be NULL when the model takes no input. */
typedef void (*ekf_filter_model_fn)(const float *x, const float *u, float dt, float *x_next, void *ctx);
/* Your sensor: z = h(x). */
typedef void (*ekf_filter_measure_fn)(const float *x, float *z, void *ctx);
/* Optional Jacobians, row-major: F is n x n (df/dx), H is m x n (dh/dx). Leave NULL and the
 * filter differentiates f and h numerically with jacobian_step. */
typedef void (*ekf_filter_model_jacobian_fn)(const float *x, const float *u, float dt, float *F, void *ctx);
typedef void (*ekf_filter_measure_jacobian_fn)(const float *x, float *H, void *ctx);

struct ekf_filter {
	int   n;                                              /* states, 1..EKF_FILTER_MAX_STATES */
	int   m;                                              /* measurements, 1..EKF_FILTER_MAX_MEAS */
	float x[EKF_FILTER_MAX_STATES];                       /* the estimate: read it */
	float P[EKF_FILTER_MAX_STATES * EKF_FILTER_MAX_STATES];   /* its covariance, n x n row-major */
	float Q[EKF_FILTER_MAX_STATES * EKF_FILTER_MAX_STATES];   /* process noise, n x n */
	float R[EKF_FILTER_MAX_MEAS * EKF_FILTER_MAX_MEAS];       /* measurement noise, m x m */
	ekf_filter_model_fn            f;
	ekf_filter_model_jacobian_fn   F;    /* may be NULL */
	ekf_filter_measure_fn          h;
	ekf_filter_measure_jacobian_fn H;    /* may be NULL */
	void *ctx;                           /* handed to every callback */
	float jacobian_step;                 /* finite-difference step; 0 selects 1e-4 */
	int   q_per_second;                  /* nonzero: Q is a rate, added as Q * dt */
	uint32_t marker;                     /* set by init, checked by every call */
};

/* Sizes, initial state and covariances in; f, h (and F, H, ctx) are set on the struct afterwards.
 * x0, P0, Q, R are copied. P0, Q and R must be symmetric within 1e-6 and finite. */
enum ekf_filter_status ekf_filter_init(struct ekf_filter *filter, int n, int m,
				       const float *x0, const float *P0,
				       const float *Q, const float *R);

/* Time update: x = f(x, u, dt), P = F P F' + Q (Q * dt with q_per_second).
 * dt must be finite and > 0. */
enum ekf_filter_status ekf_filter_predict(struct ekf_filter *filter, const float *u, float dt);

/* Measurement update with z (m values). Joseph form, so P stays symmetric and positive.
 * innovation may be NULL; otherwise receives z - h(x) before the update, m values. */
enum ekf_filter_status ekf_filter_update(struct ekf_filter *filter, const float *z, float *innovation);

/* Restart from a state and covariance without touching Q, R or the model. */
enum ekf_filter_status ekf_filter_reset(struct ekf_filter *filter, const float *x0, const float *P0);

#ifdef __cplusplus
}
#endif

#endif
