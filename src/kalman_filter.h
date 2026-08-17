#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KALMAN_FILTER_DEFAULT_PROCESS_NOISE 0.01f
#define KALMAN_FILTER_DEFAULT_MEASUREMENT_NOISE 1.0f
#define KALMAN_FILTER_INITIALIZER { 0 }

enum kalman_filter_status {
	KALMAN_FILTER_OK = 0,
	KALMAN_FILTER_ERR_NULL = -1,
	KALMAN_FILTER_ERR_NOT_FINITE = -2,
	KALMAN_FILTER_ERR_INVALID_VALUE = -3,
	KALMAN_FILTER_ERR_NOT_INITIALIZED = -4,
	KALMAN_FILTER_ERR_NUMERIC = -5,
};

struct kalman_filter {
	float estimate;
	float covariance;
	float process_noise;
	float measurement_noise;
	uint32_t marker;
	uint8_t has_estimate;
};

enum kalman_filter_status
kalman_filter_init(struct kalman_filter *filter);

enum kalman_filter_status
kalman_filter_init_with_noise(struct kalman_filter *filter,
			      float process_noise,
			      float measurement_noise);

enum kalman_filter_status
kalman_filter_reset(struct kalman_filter *filter);

enum kalman_filter_status
kalman_filter_update(struct kalman_filter *filter,
		     float measurement,
		     float *result);

#ifdef __cplusplus
}
#endif

#endif
