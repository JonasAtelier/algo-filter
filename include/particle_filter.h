#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * struct particle_filter - State for a one-dimensional particle filter.
 * @particles: Caller-owned particle storage.
 * @weights: Caller-owned normalized-weight storage.
 * @scratch: Caller-owned temporary storage used during calculations.
 * @particle_count: Number of elements in each storage array.
 * @process_noise: Prediction noise standard deviation, in state units.
 * @measurement_noise: Measurement noise standard deviation, in state units.
 * @estimate: Most recent state estimate.
 * @random_state: Internal pseudo-random generator state.
 *
 * Applications may read these fields, but should use the public functions to
 * change the filter. All three storage arrays must remain valid for the
 * lifetime of the filter. The arrays must not overlap.
 */
struct particle_filter {
	float *particles;
	float *weights;
	float *scratch;
	size_t particle_count;
	float process_noise;
	float measurement_noise;
	float estimate;
	uint32_t random_state;
};

/**
 * particle_filter_init - Initialize a scalar particle filter.
 * @filter: Filter state to initialize.
 * @particles: Storage for @particle_count particle values.
 * @weights: Storage for @particle_count particle weights.
 * @scratch: Storage for @particle_count temporary values.
 * @particle_count: Number of particles; must be at least two.
 * @initial_value: Center of the initial state distribution.
 * @initial_spread: Initial standard deviation; must be zero or greater.
 * @process_noise: Prediction standard deviation; must be zero or greater.
 * @measurement_noise: Measurement standard deviation; must be greater than
 * zero.
 * @seed: Seed for the internal pseudo-random generator. Zero is accepted.
 *
 * The three storage arrays must not overlap.
 *
 * Return: 0 on success or -1 if an argument is invalid.
 */
int particle_filter_init(struct particle_filter *filter, float *particles,
			 float *weights, float *scratch,
			 size_t particle_count, float initial_value,
			 float initial_spread, float process_noise,
			 float measurement_noise, uint32_t seed);

/**
 * particle_filter_reset - Replace the particle distribution.
 * @filter: Initialized filter.
 * @value: Center of the new state distribution.
 * @spread: New standard deviation; must be zero or greater.
 *
 * Existing noise settings and the random sequence are retained.
 *
 * Return: 0 on success or -1 if an argument is invalid.
 */
int particle_filter_reset(struct particle_filter *filter, float value,
			  float spread);

/**
 * particle_filter_predict - Apply movement and process noise.
 * @filter: Initialized filter.
 * @movement: Known state change since the previous prediction.
 *
 * The state and @movement must use the same units.
 *
 * Return: 0 on success or -1 if an argument is invalid.
 */
int particle_filter_predict(struct particle_filter *filter, float movement);

/**
 * particle_filter_update - Incorporate one measurement.
 * @filter: Initialized filter.
 * @measurement: Observed state value, in the same units as the particles.
 *
 * Weights are normalized after the measurement. The function automatically
 * performs systematic resampling when the effective particle count falls
 * below half of the configured count.
 *
 * Return: 0 on success or -1 if an argument is invalid.
 */
int particle_filter_update(struct particle_filter *filter, float measurement);

/**
 * particle_filter_get_estimate - Read the most recent state estimate.
 * @filter: Initialized filter.
 *
 * Return: Weighted state estimate, or 0.0f if @filter is invalid.
 */
float particle_filter_get_estimate(const struct particle_filter *filter);

/**
 * particle_filter_get_effective_count - Read particle diversity.
 * @filter: Initialized filter.
 *
 * Return: Effective particle count in [1, particle_count], or 0.0f if
 * @filter is invalid. Equal weights produce particle_count.
 */
float particle_filter_get_effective_count(
				const struct particle_filter *filter);

#ifdef __cplusplus
}
#endif

#endif
