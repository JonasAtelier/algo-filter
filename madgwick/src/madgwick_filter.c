/*
 * Madgwick's gradient-descent orientation filter (Sebastian Madgwick, 2010,
 * "An efficient orientation filter for inertial and inertial/magnetic sensor
 * arrays"). Each step forms the quaternion derivative implied by the gyro,
 * subtracts a normalised gradient-descent step on the accelerometer (and
 * magnetometer) error, integrates, and renormalises.
 *
 * See include/madgwick_filter.h for the frame convention. It is assumed
 * everywhere below.
 */
#include <math.h>

#include "madgwick_filter.h"

#define DEFAULT_BETA		0.1f
#define RAD_TO_DEG		57.29577951308232f

/*
 * Below this, the gradient carries no usable direction and normalising it
 * would divide by zero. The widely copied reference implementation does not
 * check, so a perfectly level and still sensor -- where the accelerometer
 * error and its gradient are both exactly zero -- fills the quaternion with
 * NaN. Skipping the correction for that step is the whole fix: with no error
 * to correct there is nothing to apply anyway.
 */
#define GRADIENT_EPSILON	1e-12f

static int finite3(float x, float y, float z)
{
	return isfinite(x) && isfinite(y) && isfinite(z);
}

static float safe_beta(float beta)
{
	if (!isfinite(beta) || beta < 0.0f)
		return DEFAULT_BETA;

	return beta;
}

/*
 * The fields are public, so a caller can leave the state unusable. Repair it
 * on the way in rather than propagating NaN, the way the sibling filters do.
 */
static void repair_state(struct madgwick_filter *filter)
{
	float norm_squared;

	filter->beta = safe_beta(filter->beta);

	norm_squared = filter->q[0] * filter->q[0] +
		       filter->q[1] * filter->q[1] +
		       filter->q[2] * filter->q[2] +
		       filter->q[3] * filter->q[3];
	if (isfinite(norm_squared) && norm_squared > GRADIENT_EPSILON)
		return;

	filter->q[0] = 1.0f;
	filter->q[1] = 0.0f;
	filter->q[2] = 0.0f;
	filter->q[3] = 0.0f;
}

static void madgwick_step(struct madgwick_filter *filter,
			  float gx, float gy, float gz,
			  float ax, float ay, float az,
			  float mx, float my, float mz,
			  int use_mag, float dt_s)
{
	float q0 = filter->q[0];
	float q1 = filter->q[1];
	float q2 = filter->q[2];
	float q3 = filter->q[3];
	float qdot0;
	float qdot1;
	float qdot2;
	float qdot3;
	float s0;
	float s1;
	float s2;
	float s3;
	float f0;
	float f1;
	float f2;
	float hx;
	float hy;
	float hz;
	float bx;
	float bz;
	float norm;

	/* Rate of change from the gyro alone: qdot = 0.5 * q * (0, gx, gy, gz). */
	qdot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qdot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qdot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qdot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	/* Only the direction of the accelerometer matters. */
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm > 0.0f) {
		ax /= norm;
		ay /= norm;
		az /= norm;

		/*
		 * Accelerometer error: where the estimate says the earth z
		 * axis should be in the sensor frame, minus where the
		 * accelerometer says it is.
		 */
		f0 = 2.0f * (q1 * q3 - q0 * q2) - ax;
		f1 = 2.0f * (q0 * q1 + q2 * q3) - ay;
		f2 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;

		/* Gradient of half the squared error: J transpose times f. */
		s0 = -2.0f * q2 * f0 + 2.0f * q1 * f1;
		s1 = 2.0f * q3 * f0 + 2.0f * q0 * f1 - 4.0f * q1 * f2;
		s2 = -2.0f * q0 * f0 + 2.0f * q3 * f1 - 4.0f * q2 * f2;
		s3 = 2.0f * q1 * f0 + 2.0f * q2 * f1;

		if (use_mag) {
			norm = sqrtf(mx * mx + my * my + mz * mz);
			mx /= norm;
			my /= norm;
			mz /= norm;

			/*
			 * The earth-frame field the estimate implies, flattened
			 * into the x-z plane. Recomputing the reference field
			 * b = (bx, 0, bz) every step is what stops magnetic
			 * distortion in the horizontal plane from tipping roll
			 * and pitch: it can only disturb yaw.
			 */
			hx = 2.0f * (mx * (0.5f - q2 * q2 - q3 * q3) +
				     my * (q1 * q2 - q0 * q3) +
				     mz * (q1 * q3 + q0 * q2));
			hy = 2.0f * (mx * (q1 * q2 + q0 * q3) +
				     my * (0.5f - q1 * q1 - q3 * q3) +
				     mz * (q2 * q3 - q0 * q1));
			hz = 2.0f * (mx * (q1 * q3 - q0 * q2) +
				     my * (q2 * q3 + q0 * q1) +
				     mz * (0.5f - q1 * q1 - q2 * q2));
			bx = sqrtf(hx * hx + hy * hy);
			bz = hz;

			f0 = 2.0f * bx * (0.5f - q2 * q2 - q3 * q3) +
			     2.0f * bz * (q1 * q3 - q0 * q2) - mx;
			f1 = 2.0f * bx * (q1 * q2 - q0 * q3) +
			     2.0f * bz * (q0 * q1 + q2 * q3) - my;
			f2 = 2.0f * bx * (q0 * q2 + q1 * q3) +
			     2.0f * bz * (0.5f - q1 * q1 - q2 * q2) - mz;

			s0 += -2.0f * bz * q2 * f0 +
			      (-2.0f * bx * q3 + 2.0f * bz * q1) * f1 +
			      2.0f * bx * q2 * f2;
			s1 += 2.0f * bz * q3 * f0 +
			      (2.0f * bx * q2 + 2.0f * bz * q0) * f1 +
			      (2.0f * bx * q3 - 4.0f * bz * q1) * f2;
			s2 += (-4.0f * bx * q2 - 2.0f * bz * q0) * f0 +
			      (2.0f * bx * q1 + 2.0f * bz * q3) * f1 +
			      (2.0f * bx * q0 - 4.0f * bz * q2) * f2;
			s3 += (-4.0f * bx * q3 + 2.0f * bz * q1) * f0 +
			      (-2.0f * bx * q0 + 2.0f * bz * q2) * f1 +
			      2.0f * bx * q1 * f2;
		}

		norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
		if (norm > GRADIENT_EPSILON) {
			qdot0 -= filter->beta * s0 / norm;
			qdot1 -= filter->beta * s1 / norm;
			qdot2 -= filter->beta * s2 / norm;
			qdot3 -= filter->beta * s3 / norm;
		}
	}

	q0 += qdot0 * dt_s;
	q1 += qdot1 * dt_s;
	q2 += qdot2 * dt_s;
	q3 += qdot3 * dt_s;

	norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	if (!isfinite(norm) || norm <= 0.0f)
		return;		/* overflowed; keep the last good estimate */

	filter->q[0] = q0 / norm;
	filter->q[1] = q1 / norm;
	filter->q[2] = q2 / norm;
	filter->q[3] = q3 / norm;
}

void madgwick_filter_init(struct madgwick_filter *filter, float beta)
{
	if (!filter)
		return;

	filter->q[0] = 1.0f;
	filter->q[1] = 0.0f;
	filter->q[2] = 0.0f;
	filter->q[3] = 0.0f;
	filter->beta = safe_beta(beta);
}

void madgwick_filter_reset(struct madgwick_filter *filter, const float q[4])
{
	float norm;

	if (!filter)
		return;

	filter->beta = safe_beta(filter->beta);

	if (!q) {
		filter->q[0] = 1.0f;
		filter->q[1] = 0.0f;
		filter->q[2] = 0.0f;
		filter->q[3] = 0.0f;
		return;
	}

	if (!isfinite(q[0]) || !finite3(q[1], q[2], q[3]))
		return;

	norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (!isfinite(norm) || norm <= 0.0f)
		return;

	filter->q[0] = q[0] / norm;
	filter->q[1] = q[1] / norm;
	filter->q[2] = q[2] / norm;
	filter->q[3] = q[3] / norm;
}

void madgwick_filter_update_imu(struct madgwick_filter *filter,
				float gx, float gy, float gz,
				float ax, float ay, float az, float dt_s)
{
	if (!filter)
		return;

	repair_state(filter);

	if (!isfinite(dt_s) || dt_s <= 0.0f ||
	    !finite3(gx, gy, gz) || !finite3(ax, ay, az))
		return;

	madgwick_step(filter, gx, gy, gz, ax, ay, az, 0.0f, 0.0f, 0.0f,
		      0, dt_s);
}

void madgwick_filter_update(struct madgwick_filter *filter,
			    float gx, float gy, float gz,
			    float ax, float ay, float az,
			    float mx, float my, float mz, float dt_s)
{
	if (!filter)
		return;

	repair_state(filter);

	if (!finite3(mx, my, mz))
		return;

	/* Nothing to take a heading from; the gyro and accelerometer still can. */
	if (mx == 0.0f && my == 0.0f && mz == 0.0f) {
		madgwick_filter_update_imu(filter, gx, gy, gz, ax, ay, az,
					   dt_s);
		return;
	}

	if (!isfinite(dt_s) || dt_s <= 0.0f ||
	    !finite3(gx, gy, gz) || !finite3(ax, ay, az))
		return;

	madgwick_step(filter, gx, gy, gz, ax, ay, az, mx, my, mz, 1, dt_s);
}

void madgwick_filter_euler(const struct madgwick_filter *filter,
			   float *roll_deg, float *pitch_deg, float *yaw_deg)
{
	float q0;
	float q1;
	float q2;
	float q3;
	float sin_pitch;

	if (!filter)
		return;

	q0 = filter->q[0];
	q1 = filter->q[1];
	q2 = filter->q[2];
	q3 = filter->q[3];

	if (roll_deg)
		*roll_deg = atan2f(2.0f * (q0 * q1 + q2 * q3),
				   1.0f - 2.0f * (q1 * q1 + q2 * q2)) *
			    RAD_TO_DEG;

	if (pitch_deg) {
		/* Rounding can push this a hair outside asinf's domain. */
		sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
		if (sin_pitch > 1.0f)
			sin_pitch = 1.0f;
		else if (sin_pitch < -1.0f)
			sin_pitch = -1.0f;

		*pitch_deg = asinf(sin_pitch) * RAD_TO_DEG;
	}

	if (yaw_deg)
		*yaw_deg = atan2f(2.0f * (q0 * q3 + q1 * q2),
				  1.0f - 2.0f * (q2 * q2 + q3 * q3)) *
			   RAD_TO_DEG;
}
