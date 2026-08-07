#include <math.h>

#include "complementary_filter.h"

#define DEFAULT_ALPHA	0.98f
#define SAFE_ANGLE_DEG	0.0f

static float clamp_alpha(float alpha)
{
	if (!isfinite(alpha))
		return DEFAULT_ALPHA;
	if (alpha < 0.0f)
		return 0.0f;
	if (alpha > 1.0f)
		return 1.0f;

	return alpha;
}

static float safe_angle(float angle_deg)
{
	if (!isfinite(angle_deg))
		return SAFE_ANGLE_DEG;

	return angle_deg;
}

static float wrap_angle_error(float angle_deg)
{
	float wrapped_deg;

	wrapped_deg = fmodf(angle_deg, 360.0f);
	if (wrapped_deg >= 180.0f)
		wrapped_deg -= 360.0f;
	else if (wrapped_deg < -180.0f)
		wrapped_deg += 360.0f;

	return wrapped_deg;
}

void complementary_filter_init(struct complementary_filter *filter,
			       float initial_angle_deg, float alpha)
{
	if (!filter)
		return;

	filter->angle_deg = safe_angle(initial_angle_deg);
	filter->alpha = clamp_alpha(alpha);
}

void complementary_filter_reset(struct complementary_filter *filter,
				float angle_deg)
{
	if (!filter)
		return;

	if (isfinite(angle_deg))
		filter->angle_deg = angle_deg;
	else
		filter->angle_deg = safe_angle(filter->angle_deg);
}

float complementary_filter_update(struct complementary_filter *filter,
				  float gyro_rate_dps,
				  float accel_angle_deg, float dt_s)
{
	float accel_wrapped_deg;
	float correction_deg;
	float gyro_delta_deg;
	float gyro_angle_deg;
	float gyro_wrapped_deg;
	float previous_angle_deg;
	float previous_wrapped_deg;

	if (!filter)
		return 0.0f;

	previous_angle_deg = safe_angle(filter->angle_deg);
	filter->angle_deg = previous_angle_deg;
	filter->alpha = clamp_alpha(filter->alpha);

	if (!isfinite(dt_s) || dt_s <= 0.0f ||
	    !isfinite(gyro_rate_dps) || !isfinite(accel_angle_deg))
		return previous_angle_deg;

	if (filter->alpha == 0.0f) {
		accel_wrapped_deg = wrap_angle_error(accel_angle_deg);
		previous_wrapped_deg = wrap_angle_error(previous_angle_deg);
		correction_deg = wrap_angle_error(accel_wrapped_deg -
						  previous_wrapped_deg);
		filter->angle_deg = previous_angle_deg + correction_deg;
		return filter->angle_deg;
	}

	gyro_delta_deg = gyro_rate_dps * dt_s;
	if (!isfinite(gyro_delta_deg))
		return previous_angle_deg;

	gyro_angle_deg = previous_angle_deg + gyro_delta_deg;
	if (!isfinite(gyro_angle_deg))
		return previous_angle_deg;

	if (filter->alpha == 1.0f) {
		filter->angle_deg = gyro_angle_deg;
		return filter->angle_deg;
	}

	accel_wrapped_deg = wrap_angle_error(accel_angle_deg);
	gyro_wrapped_deg = wrap_angle_error(gyro_angle_deg);
	correction_deg = wrap_angle_error(accel_wrapped_deg -
					  gyro_wrapped_deg);
	filter->angle_deg = gyro_angle_deg +
			    (1.0f - filter->alpha) * correction_deg;

	return filter->angle_deg;
}
