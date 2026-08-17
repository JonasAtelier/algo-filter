#include "kalman_filter.h"

#include <float.h>
#include <math.h>

#define KALMAN_FILTER_MARKER UINT32_C(0x4b464c54)
#define KALMAN_FILTER_INITIAL_COVARIANCE 1.0f

static int kalman_filter_is_valid(const struct kalman_filter *filter)
{
	return filter->marker == KALMAN_FILTER_MARKER &&
	       isfinite(filter->estimate) &&
	       isfinite(filter->covariance) &&
	       isfinite(filter->process_noise) &&
	       isfinite(filter->measurement_noise) &&
	       filter->covariance >= 0.0f &&
	       filter->process_noise >= 0.0f &&
	       filter->measurement_noise > 0.0f &&
	       filter->has_estimate <= 1;
}

static int
kalman_filter_result_aliases(const struct kalman_filter *filter,
			     const float *result)
{
	return result == &filter->estimate ||
	       result == &filter->covariance ||
	       result == &filter->process_noise ||
	       result == &filter->measurement_noise;
}

static enum kalman_filter_status
kalman_filter_check_noise(float process_noise, float measurement_noise)
{
	if (!isfinite(process_noise) || !isfinite(measurement_noise))
		return KALMAN_FILTER_ERR_NOT_FINITE;

	if (process_noise < 0.0f || measurement_noise <= 0.0f)
		return KALMAN_FILTER_ERR_INVALID_VALUE;

	return KALMAN_FILTER_OK;
}

enum kalman_filter_status
kalman_filter_init(struct kalman_filter *filter)
{
	return kalman_filter_init_with_noise(
		filter, KALMAN_FILTER_DEFAULT_PROCESS_NOISE,
		KALMAN_FILTER_DEFAULT_MEASUREMENT_NOISE);
}

enum kalman_filter_status
kalman_filter_init_with_noise(struct kalman_filter *filter,
			      float process_noise,
			      float measurement_noise)
{
	struct kalman_filter next;
	enum kalman_filter_status status;

	if (!filter)
		return KALMAN_FILTER_ERR_NULL;

	status = kalman_filter_check_noise(process_noise, measurement_noise);
	if (status != KALMAN_FILTER_OK)
		return status;

	next.estimate = 0.0f;
	next.covariance = KALMAN_FILTER_INITIAL_COVARIANCE;
	next.process_noise = process_noise;
	next.measurement_noise = measurement_noise;
	next.marker = KALMAN_FILTER_MARKER;
	next.has_estimate = 0;
	*filter = next;

	return KALMAN_FILTER_OK;
}

enum kalman_filter_status
kalman_filter_reset(struct kalman_filter *filter)
{
	if (!filter)
		return KALMAN_FILTER_ERR_NULL;

	if (!kalman_filter_is_valid(filter))
		return KALMAN_FILTER_ERR_NOT_INITIALIZED;

	filter->estimate = 0.0f;
	filter->covariance = KALMAN_FILTER_INITIAL_COVARIANCE;
	filter->has_estimate = 0;

	return KALMAN_FILTER_OK;
}

enum kalman_filter_status
kalman_filter_update(struct kalman_filter *filter,
		     float measurement,
		     float *result)
{
	struct kalman_filter next;
	double predicted_covariance;
	double innovation_covariance;
	double gain;
	double estimate;
	double covariance;

	if (!filter || !result)
		return KALMAN_FILTER_ERR_NULL;

	if (kalman_filter_result_aliases(filter, result))
		return KALMAN_FILTER_ERR_INVALID_VALUE;

	if (!kalman_filter_is_valid(filter))
		return KALMAN_FILTER_ERR_NOT_INITIALIZED;

	if (!isfinite(measurement))
		return KALMAN_FILTER_ERR_NOT_FINITE;

	if (!filter->has_estimate) {
		next = *filter;
		next.estimate = measurement;
		next.has_estimate = 1;
		*filter = next;
		*result = measurement;
		return KALMAN_FILTER_OK;
	}

	predicted_covariance = (double)filter->covariance +
			       (double)filter->process_noise;
	innovation_covariance = predicted_covariance +
				(double)filter->measurement_noise;

	if (!isfinite(predicted_covariance) ||
	    !isfinite(innovation_covariance) ||
	    innovation_covariance <= 0.0)
		return KALMAN_FILTER_ERR_NUMERIC;

	gain = predicted_covariance / innovation_covariance;
	estimate = (double)filter->estimate +
		   gain * ((double)measurement - (double)filter->estimate);
	covariance = (1.0 - gain) * (1.0 - gain) *
		     predicted_covariance +
		     gain * gain * (double)filter->measurement_noise;

	if (!isfinite(gain) || !isfinite(estimate) ||
	    !isfinite(covariance) || covariance < 0.0 ||
	    estimate > FLT_MAX || estimate < -FLT_MAX ||
	    covariance > FLT_MAX)
		return KALMAN_FILTER_ERR_NUMERIC;

	next = *filter;
	next.estimate = (float)estimate;
	next.covariance = (float)covariance;
	*filter = next;
	*result = next.estimate;

	return KALMAN_FILTER_OK;
}
