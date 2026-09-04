#include "ekf_filter.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#define EKF_FILTER_MARKER		UINT32_C(0x454b464c)
#define EKF_FILTER_DEFAULT_JACOBIAN_STEP 1e-4
#define EKF_FILTER_SYMMETRY_TOLERANCE	1e-6
#define EKF_FILTER_PIVOT_MIN		1e-12

#define EKF_FILTER_NN	(EKF_FILTER_MAX_STATES * EKF_FILTER_MAX_STATES)
#define EKF_FILTER_MN	(EKF_FILTER_MAX_MEAS * EKF_FILTER_MAX_STATES)
#define EKF_FILTER_MM	(EKF_FILTER_MAX_MEAS * EKF_FILTER_MAX_MEAS)

/* ---- small matrix helpers -------------------------------------------
 *
 * Row-major throughout: M[row * cols + col]. The destination must not be one
 * of the sources.
 */

/* C(ra x cb) = A(ra x ca) * B(ca x cb) */
static void ekf_mat_mul(int ra, int ca, int cb, const double *A, const double *B,
			double *C)
{
	int i, j, k;

	for (i = 0; i < ra; i++)
		for (j = 0; j < cb; j++) {
			double s = 0.0;

			for (k = 0; k < ca; k++)
				s += A[i * ca + k] * B[k * cb + j];
			C[i * cb + j] = s;
		}
}

/* C(ra x rb) = A(ra x c) * B(rb x c)' */
static void ekf_mat_mul_bt(int ra, int c, int rb, const double *A,
			   const double *B, double *C)
{
	int i, j, k;

	for (i = 0; i < ra; i++)
		for (j = 0; j < rb; j++) {
			double s = 0.0;

			for (k = 0; k < c; k++)
				s += A[i * c + k] * B[j * c + k];
			C[i * rb + j] = s;
		}
}

/* Gauss-Jordan with partial pivoting: X <- M^-1 X over nrhs columns.
 * M (m x m) and X (m x nrhs) are both destroyed. Returns 0, or -1 if M is
 * singular, in which case X holds partial work and must be discarded. */
static int ekf_gj_solve(int m, int nrhs, double *M, double *X)
{
	int i, j, k;

	for (k = 0; k < m; k++) {
		int piv = k;
		double max = fabs(M[k * m + k]), d;

		for (i = k + 1; i < m; i++)
			if (fabs(M[i * m + k]) > max) {
				max = fabs(M[i * m + k]);
				piv = i;
			}
		if (max < EKF_FILTER_PIVOT_MIN)
			return -1;			/* singular */

		if (piv != k) {
			for (j = 0; j < m; j++) {
				double t = M[k * m + j];

				M[k * m + j] = M[piv * m + j];
				M[piv * m + j] = t;
			}
			for (j = 0; j < nrhs; j++) {
				double t = X[k * nrhs + j];

				X[k * nrhs + j] = X[piv * nrhs + j];
				X[piv * nrhs + j] = t;
			}
		}

		d = M[k * m + k];
		for (j = 0; j < m; j++)
			M[k * m + j] /= d;
		for (j = 0; j < nrhs; j++)
			X[k * nrhs + j] /= d;

		for (i = 0; i < m; i++) {
			double f = M[i * m + k];

			if (i == k || f == 0.0)
				continue;
			for (j = 0; j < m; j++)
				M[i * m + j] -= f * M[k * m + j];
			for (j = 0; j < nrhs; j++)
				X[i * nrhs + j] -= f * X[k * nrhs + j];
		}
	}

	return 0;
}

/* ---- validation ------------------------------------------------------ */

static int ekf_float_finite(const float *v, int count)
{
	int i;

	for (i = 0; i < count; i++)
		if (!isfinite(v[i]))
			return 0;

	return 1;
}

/* Finite and small enough to survive the trip back into float storage. */
static int ekf_storable(const double *v, int count)
{
	int i;

	for (i = 0; i < count; i++)
		if (!isfinite(v[i]) || v[i] > (double)FLT_MAX ||
		    v[i] < -(double)FLT_MAX)
			return 0;

	return 1;
}

static int ekf_symmetric(const float *M, int n)
{
	int i, j;

	for (i = 0; i < n; i++)
		for (j = i + 1; j < n; j++)
			if (fabs((double)M[i * n + j] - (double)M[j * n + i]) >
			    EKF_FILTER_SYMMETRY_TOLERANCE)
				return 0;

	return 1;
}

static void ekf_to_double(const float *src, double *dst, int count)
{
	int i;

	for (i = 0; i < count; i++)
		dst[i] = (double)src[i];
}

static void ekf_to_float(const double *src, float *dst, int count)
{
	int i;

	for (i = 0; i < count; i++)
		dst[i] = (float)src[i];
}

/* P is symmetric in exact arithmetic; forcing it stops rounding from slowly
 * tilting it, which is what keeps a long run stable. */
static void ekf_symmetrise(double *M, int n)
{
	int i, j;

	for (i = 0; i < n; i++)
		for (j = i + 1; j < n; j++) {
			double mean = 0.5 * (M[i * n + j] + M[j * n + i]);

			M[i * n + j] = mean;
			M[j * n + i] = mean;
		}
}

/* Everything a stored filter must satisfy before it is used again. The model
 * callbacks are checked separately, since reset does not need them. */
static enum ekf_filter_status ekf_check_state(const struct ekf_filter *filter)
{
	if (!filter)
		return EKF_FILTER_ERR_NULL;

	if (filter->marker != EKF_FILTER_MARKER)
		return EKF_FILTER_ERR_NOT_INITIALIZED;

	if (filter->n < 1 || filter->n > EKF_FILTER_MAX_STATES ||
	    filter->m < 1 || filter->m > EKF_FILTER_MAX_MEAS)
		return EKF_FILTER_ERR_INVALID_VALUE;

	if (!ekf_float_finite(filter->x, filter->n) ||
	    !ekf_float_finite(filter->P, filter->n * filter->n) ||
	    !ekf_float_finite(filter->Q, filter->n * filter->n) ||
	    !ekf_float_finite(filter->R, filter->m * filter->m))
		return EKF_FILTER_ERR_NOT_FINITE;

	return EKF_FILTER_OK;
}

/* ---- numerical Jacobians --------------------------------------------
 *
 * Central differences. The step is scaled by the size of the state so it stays
 * representable once the perturbed value is stored back into a float, and the
 * divisor is the difference the two floats actually hold rather than the step
 * that was asked for. Without both, a state of a few hundred loses most of a
 * 1e-4 step to rounding and the derivative comes out several percent wrong.
 */
static double ekf_jacobian_step(const struct ekf_filter *filter)
{
	if (isfinite(filter->jacobian_step) && filter->jacobian_step > 0.0f)
		return (double)filter->jacobian_step;

	return EKF_FILTER_DEFAULT_JACOBIAN_STEP;
}

static void ekf_numeric_model_jacobian(const struct ekf_filter *filter,
				       const float *u, float dt, double *F)
{
	float plus[EKF_FILTER_MAX_STATES];
	float minus[EKF_FILTER_MAX_STATES];
	float f_plus[EKF_FILTER_MAX_STATES];
	float f_minus[EKF_FILTER_MAX_STATES];
	double base = ekf_jacobian_step(filter);
	int n = filter->n;
	int i, j;

	for (j = 0; j < n; j++) {
		double step = base * (1.0 + fabs((double)filter->x[j]));
		double taken;

		for (i = 0; i < n; i++) {
			plus[i] = filter->x[i];
			minus[i] = filter->x[i];
		}
		plus[j] = (float)((double)filter->x[j] + step);
		minus[j] = (float)((double)filter->x[j] - step);
		taken = (double)plus[j] - (double)minus[j];

		filter->f(plus, u, dt, f_plus, filter->ctx);
		filter->f(minus, u, dt, f_minus, filter->ctx);

		for (i = 0; i < n; i++)
			F[i * n + j] = taken != 0.0 ?
				((double)f_plus[i] - (double)f_minus[i]) / taken :
				(double)(i == j);
	}
}

static void ekf_numeric_measure_jacobian(const struct ekf_filter *filter,
					 double *H)
{
	float plus[EKF_FILTER_MAX_STATES];
	float minus[EKF_FILTER_MAX_STATES];
	float h_plus[EKF_FILTER_MAX_MEAS];
	float h_minus[EKF_FILTER_MAX_MEAS];
	double base = ekf_jacobian_step(filter);
	int n = filter->n;
	int m = filter->m;
	int i, j;

	for (j = 0; j < n; j++) {
		double step = base * (1.0 + fabs((double)filter->x[j]));
		double taken;

		for (i = 0; i < n; i++) {
			plus[i] = filter->x[i];
			minus[i] = filter->x[i];
		}
		plus[j] = (float)((double)filter->x[j] + step);
		minus[j] = (float)((double)filter->x[j] - step);
		taken = (double)plus[j] - (double)minus[j];

		filter->h(plus, h_plus, filter->ctx);
		filter->h(minus, h_minus, filter->ctx);

		for (i = 0; i < m; i++)
			H[i * n + j] = taken != 0.0 ?
				((double)h_plus[i] - (double)h_minus[i]) / taken :
				0.0;
	}
}

/* ---- the public filter ----------------------------------------------- */

enum ekf_filter_status ekf_filter_init(struct ekf_filter *filter, int n, int m,
				       const float *x0, const float *P0,
				       const float *Q, const float *R)
{
	int i;

	if (!filter || !x0 || !P0 || !Q || !R)
		return EKF_FILTER_ERR_NULL;

	if (n < 1 || n > EKF_FILTER_MAX_STATES ||
	    m < 1 || m > EKF_FILTER_MAX_MEAS)
		return EKF_FILTER_ERR_INVALID_VALUE;

	if (!ekf_float_finite(x0, n) || !ekf_float_finite(P0, n * n) ||
	    !ekf_float_finite(Q, n * n) || !ekf_float_finite(R, m * m))
		return EKF_FILTER_ERR_NOT_FINITE;

	if (!ekf_symmetric(P0, n) || !ekf_symmetric(Q, n) ||
	    !ekf_symmetric(R, m))
		return EKF_FILTER_ERR_INVALID_VALUE;

	for (i = 0; i < EKF_FILTER_MAX_STATES; i++)
		filter->x[i] = 0.0f;
	for (i = 0; i < EKF_FILTER_NN; i++) {
		filter->P[i] = 0.0f;
		filter->Q[i] = 0.0f;
	}
	for (i = 0; i < EKF_FILTER_MM; i++)
		filter->R[i] = 0.0f;

	for (i = 0; i < n; i++)
		filter->x[i] = x0[i];
	for (i = 0; i < n * n; i++) {
		filter->P[i] = P0[i];
		filter->Q[i] = Q[i];
	}
	for (i = 0; i < m * m; i++)
		filter->R[i] = R[i];

	filter->n = n;
	filter->m = m;
	filter->f = NULL;
	filter->F = NULL;
	filter->h = NULL;
	filter->H = NULL;
	filter->ctx = NULL;
	filter->jacobian_step = 0.0f;
	filter->marker = EKF_FILTER_MARKER;

	return EKF_FILTER_OK;
}

enum ekf_filter_status ekf_filter_predict(struct ekf_filter *filter,
					  const float *u, float dt)
{
	double F[EKF_FILTER_NN] = { 0.0 };
	double P[EKF_FILTER_NN] = { 0.0 };
	double FP[EKF_FILTER_NN] = { 0.0 };
	double next_P[EKF_FILTER_NN] = { 0.0 };
	float jacobian[EKF_FILTER_NN];
	float next_x[EKF_FILTER_MAX_STATES];
	enum ekf_filter_status status;
	int n, i;

	status = ekf_check_state(filter);
	if (status != EKF_FILTER_OK)
		return status;

	if (!filter->f || !filter->h)
		return EKF_FILTER_ERR_INVALID_VALUE;

	if (!isfinite(dt))
		return EKF_FILTER_ERR_NOT_FINITE;

	if (dt <= 0.0f)
		return EKF_FILTER_ERR_INVALID_VALUE;

	n = filter->n;

	/* F is the Jacobian at the state we are leaving, so it comes first. */
	if (filter->F) {
		filter->F(filter->x, u, dt, jacobian, filter->ctx);
		if (!ekf_float_finite(jacobian, n * n))
			return EKF_FILTER_ERR_NOT_FINITE;
		ekf_to_double(jacobian, F, n * n);
	} else {
		ekf_numeric_model_jacobian(filter, u, dt, F);
		if (!ekf_storable(F, n * n))
			return EKF_FILTER_ERR_NOT_FINITE;
	}

	filter->f(filter->x, u, dt, next_x, filter->ctx);
	if (!ekf_float_finite(next_x, n))
		return EKF_FILTER_ERR_NOT_FINITE;

	ekf_to_double(filter->P, P, n * n);
	ekf_mat_mul(n, n, n, F, P, FP);			/* FP = F P        */
	ekf_mat_mul_bt(n, n, n, FP, F, next_P);		/* next_P = F P F' */
	for (i = 0; i < n * n; i++)
		next_P[i] += (double)filter->Q[i];
	ekf_symmetrise(next_P, n);

	if (!ekf_storable(next_P, n * n))
		return EKF_FILTER_ERR_NUMERIC;

	for (i = 0; i < n; i++)
		filter->x[i] = next_x[i];
	ekf_to_float(next_P, filter->P, n * n);

	return EKF_FILTER_OK;
}

enum ekf_filter_status ekf_filter_update(struct ekf_filter *filter,
					 const float *z, float *innovation)
{
	double P[EKF_FILTER_NN] = { 0.0 };
	double KH[EKF_FILTER_NN] = { 0.0 };
	double A[EKF_FILTER_NN] = { 0.0 };
	double T[EKF_FILTER_NN] = { 0.0 };
	double next_P[EKF_FILTER_NN] = { 0.0 };
	double H[EKF_FILTER_MN] = { 0.0 };
	double HP[EKF_FILTER_MN] = { 0.0 };
	double K[EKF_FILTER_MN] = { 0.0 };
	double KR[EKF_FILTER_MN] = { 0.0 };
	double S[EKF_FILTER_MM] = { 0.0 };
	double y[EKF_FILTER_MAX_MEAS] = { 0.0 };
	double next_x[EKF_FILTER_MAX_STATES] = { 0.0 };
	float jacobian[EKF_FILTER_MN];
	float predicted[EKF_FILTER_MAX_MEAS];
	enum ekf_filter_status status;
	int n, m, i, j;

	status = ekf_check_state(filter);
	if (status != EKF_FILTER_OK)
		return status;

	if (!z)
		return EKF_FILTER_ERR_NULL;

	if (!filter->f || !filter->h)
		return EKF_FILTER_ERR_INVALID_VALUE;

	n = filter->n;
	m = filter->m;

	if (!ekf_float_finite(z, m))
		return EKF_FILTER_ERR_NOT_FINITE;

	filter->h(filter->x, predicted, filter->ctx);
	if (!ekf_float_finite(predicted, m))
		return EKF_FILTER_ERR_NOT_FINITE;

	if (filter->H) {
		filter->H(filter->x, jacobian, filter->ctx);
		if (!ekf_float_finite(jacobian, m * n))
			return EKF_FILTER_ERR_NOT_FINITE;
		ekf_to_double(jacobian, H, m * n);
	} else {
		ekf_numeric_measure_jacobian(filter, H);
		if (!ekf_storable(H, m * n))
			return EKF_FILTER_ERR_NOT_FINITE;
	}

	ekf_to_double(filter->P, P, n * n);
	ekf_mat_mul(m, n, n, H, P, HP);			/* HP = H P,  m x n */
	ekf_mat_mul_bt(m, n, m, HP, H, S);		/* S  = H P H'      */
	for (i = 0; i < m * m; i++)
		S[i] += (double)filter->R[i];

	if (!ekf_storable(S, m * m) || !ekf_storable(HP, m * n))
		return EKF_FILTER_ERR_NUMERIC;

	/* K = P H' S^-1, solved in place: the inverse is never formed. S and P
	 * are symmetric, so (P H' S^-1)' = S^-1 H P, and HP becomes that. */
	if (ekf_gj_solve(m, n, S, HP) != 0)
		return EKF_FILTER_ERR_NUMERIC;

	for (i = 0; i < n; i++)
		for (j = 0; j < m; j++)
			K[i * m + j] = HP[j * n + i];

	for (i = 0; i < m; i++)
		y[i] = (double)z[i] - (double)predicted[i];

	for (i = 0; i < n; i++) {
		next_x[i] = (double)filter->x[i];
		for (j = 0; j < m; j++)
			next_x[i] += K[i * m + j] * y[j];
	}

	/* Joseph form: next_P = (I - K H) P (I - K H)' + K R K'. */
	ekf_mat_mul(n, m, n, K, H, KH);
	for (i = 0; i < n * n; i++)
		A[i] = -KH[i];
	for (i = 0; i < n; i++)
		A[i * n + i] += 1.0;

	ekf_mat_mul(n, n, n, A, P, T);
	ekf_mat_mul_bt(n, n, n, T, A, next_P);

	ekf_to_double(filter->R, S, m * m);		/* S reused: R again */
	ekf_mat_mul(n, m, m, K, S, KR);
	ekf_mat_mul_bt(n, m, n, KR, K, T);
	for (i = 0; i < n * n; i++)
		next_P[i] += T[i];
	ekf_symmetrise(next_P, n);

	if (!ekf_storable(next_x, n) || !ekf_storable(next_P, n * n))
		return EKF_FILTER_ERR_NUMERIC;

	ekf_to_float(next_x, filter->x, n);
	ekf_to_float(next_P, filter->P, n * n);

	if (innovation)
		for (i = 0; i < m; i++)
			innovation[i] = (float)y[i];

	return EKF_FILTER_OK;
}

enum ekf_filter_status ekf_filter_reset(struct ekf_filter *filter,
					const float *x0, const float *P0)
{
	enum ekf_filter_status status;
	int n, i;

	status = ekf_check_state(filter);
	if (status != EKF_FILTER_OK)
		return status;

	if (!x0 || !P0)
		return EKF_FILTER_ERR_NULL;

	n = filter->n;

	if (!ekf_float_finite(x0, n) || !ekf_float_finite(P0, n * n))
		return EKF_FILTER_ERR_NOT_FINITE;

	if (!ekf_symmetric(P0, n))
		return EKF_FILTER_ERR_INVALID_VALUE;

	for (i = 0; i < n; i++)
		filter->x[i] = x0[i];
	for (i = 0; i < n * n; i++)
		filter->P[i] = P0[i];

	return EKF_FILTER_OK;
}
