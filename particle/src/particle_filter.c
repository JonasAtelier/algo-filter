#include <float.h>
#include <math.h>

#include "particle_filter.h"

#define PARTICLE_FILTER_DEFAULT_SEED	0x6d2b79f5U
#define PARTICLE_FILTER_TWO_PI		6.28318530717958647692

static int particle_filter_ranges_overlap(const float *first,
					   const float *second,
					   size_t particle_count)
{
	uintptr_t first_start = (uintptr_t)first;
	uintptr_t second_start = (uintptr_t)second;
	size_t bytes;

	if (particle_count > SIZE_MAX / sizeof(*first))
		return 1;

	bytes = particle_count * sizeof(*first);
	if (first_start > UINTPTR_MAX - bytes ||
	    second_start > UINTPTR_MAX - bytes)
		return 1;

	return first_start < second_start + bytes &&
	       second_start < first_start + bytes;
}

static int particle_filter_storage_is_valid(const float *particles,
					     const float *weights,
					     const float *scratch,
					     size_t particle_count)
{
	if (!particles || !weights || !scratch || particle_count < 2U)
		return 0;

	return !particle_filter_ranges_overlap(particles, weights,
					       particle_count) &&
	       !particle_filter_ranges_overlap(particles, scratch,
					       particle_count) &&
	       !particle_filter_ranges_overlap(weights, scratch,
					       particle_count);
}

static int particle_filter_is_valid(const struct particle_filter *filter)
{
	return filter &&
	       particle_filter_storage_is_valid(filter->particles,
						filter->weights,
						filter->scratch,
						filter->particle_count) &&
	       isfinite(filter->process_noise) && filter->process_noise >= 0.0f &&
	       isfinite(filter->measurement_noise) &&
	       filter->measurement_noise > 0.0f && isfinite(filter->estimate);
}

static uint32_t particle_filter_random_u32(struct particle_filter *filter)
{
	uint32_t value = filter->random_state;

	value ^= value << 13;
	value ^= value >> 17;
	value ^= value << 5;
	filter->random_state = value;

	return value;
}

static double particle_filter_random_open(struct particle_filter *filter)
{
	uint32_t value;

	value = particle_filter_random_u32(filter);
	return ((double)(value >> 8) + 1.0) / 16777217.0;
}

static float particle_filter_random_unit(struct particle_filter *filter)
{
	uint32_t value;

	value = particle_filter_random_u32(filter);
	return (float)(value >> 8) * (1.0f / 16777216.0f);
}

static double particle_filter_random_normal(struct particle_filter *filter)
{
	double first;
	double second;

	first = particle_filter_random_open(filter);
	second = particle_filter_random_open(filter);

	return sqrt(-2.0 * log(first)) *
	       cos(PARTICLE_FILTER_TWO_PI * second);
}

static int particle_filter_set_distribution(struct particle_filter *filter,
					    float value, float spread)
{
	float weight = 1.0f / (float)filter->particle_count;
	double estimate = 0.0;
	size_t index;

	for (index = 0; index < filter->particle_count; index++) {
		double candidate;

		candidate = (double)value +
			    (double)spread *
			    particle_filter_random_normal(filter);
		if (!isfinite(candidate) || candidate > FLT_MAX ||
		    candidate < -FLT_MAX)
			return -1;

		filter->scratch[index] = (float)candidate;
		estimate += candidate;
	}

	estimate /= (double)filter->particle_count;
	if (!isfinite(estimate) || estimate > FLT_MAX ||
	    estimate < -FLT_MAX)
		return -1;

	for (index = 0; index < filter->particle_count; index++) {
		filter->particles[index] = filter->scratch[index];
		filter->weights[index] = weight;
	}

	filter->estimate = (float)estimate;

	return 0;
}

static void particle_filter_resample(struct particle_filter *filter)
{
	float step = 1.0f / (float)filter->particle_count;
	float position = particle_filter_random_unit(filter) * step;
	float cumulative = filter->weights[0];
	float weight = step;
	double estimate = 0.0;
	size_t source = 0;
	size_t index;

	for (index = 0; index < filter->particle_count; index++) {
		while (position > cumulative &&
		       source + 1U < filter->particle_count) {
			source++;
			cumulative += filter->weights[source];
		}

		filter->scratch[index] = filter->particles[source];
		position += step;
	}

	for (index = 0; index < filter->particle_count; index++) {
		filter->particles[index] = filter->scratch[index];
		filter->weights[index] = weight;
		estimate += filter->particles[index];
	}

	filter->estimate =
		(float)(estimate / (double)filter->particle_count);
}

int particle_filter_init(struct particle_filter *filter, float *particles,
			 float *weights, float *scratch,
			 size_t particle_count, float initial_value,
			 float initial_spread, float process_noise,
			 float measurement_noise, uint32_t seed)
{
	struct particle_filter candidate;

	if (!filter ||
	    !particle_filter_storage_is_valid(particles, weights, scratch,
					      particle_count) ||
	    !isfinite(initial_value) ||
	    !isfinite(initial_spread) || initial_spread < 0.0f ||
	    !isfinite(process_noise) || process_noise < 0.0f ||
	    !isfinite(measurement_noise) || measurement_noise <= 0.0f)
		return -1;

	candidate.particles = particles;
	candidate.weights = weights;
	candidate.scratch = scratch;
	candidate.particle_count = particle_count;
	candidate.process_noise = process_noise;
	candidate.measurement_noise = measurement_noise;
	candidate.estimate = initial_value;
	candidate.random_state = seed ? seed : PARTICLE_FILTER_DEFAULT_SEED;

	if (particle_filter_set_distribution(&candidate, initial_value,
					     initial_spread))
		return -1;

	*filter = candidate;

	return 0;
}

int particle_filter_reset(struct particle_filter *filter, float value,
			  float spread)
{
	struct particle_filter candidate;

	if (!particle_filter_is_valid(filter) || !isfinite(value) ||
	    !isfinite(spread) || spread < 0.0f)
		return -1;

	candidate = *filter;
	if (particle_filter_set_distribution(&candidate, value, spread))
		return -1;

	filter->estimate = candidate.estimate;
	filter->random_state = candidate.random_state;

	return 0;
}

int particle_filter_predict(struct particle_filter *filter, float movement)
{
	struct particle_filter candidate;
	double estimate = 0.0;
	double weight_total = 0.0;
	size_t index;

	if (!particle_filter_is_valid(filter) || !isfinite(movement))
		return -1;

	candidate = *filter;
	for (index = 0; index < filter->particle_count; index++) {
		double predicted;

		if (!isfinite(filter->particles[index]) ||
		    !isfinite(filter->weights[index]) ||
		    filter->weights[index] < 0.0f)
			return -1;

		predicted = (double)filter->particles[index] + movement +
			    (double)filter->process_noise *
			    particle_filter_random_normal(&candidate);
		if (!isfinite(predicted) || predicted > FLT_MAX ||
		    predicted < -FLT_MAX)
			return -1;

		filter->scratch[index] = (float)predicted;
		estimate += predicted * filter->weights[index];
		weight_total += filter->weights[index];
	}

	if (!isfinite(weight_total) || weight_total <= 0.0)
		return -1;

	estimate /= weight_total;
	if (!isfinite(estimate) || estimate > FLT_MAX ||
	    estimate < -FLT_MAX)
		return -1;

	for (index = 0; index < filter->particle_count; index++)
		filter->particles[index] = filter->scratch[index];

	filter->estimate = (float)estimate;
	filter->random_state = candidate.random_state;

	return 0;
}

int particle_filter_update(struct particle_filter *filter, float measurement)
{
	double maximum_log_weight = -DBL_MAX;
	double total = 0.0;
	double estimate = 0.0;
	double normalized_total = 0.0;
	double square_total = 0.0;
	size_t index;

	if (!particle_filter_is_valid(filter) || !isfinite(measurement))
		return -1;

	for (index = 0; index < filter->particle_count; index++) {
		double difference;
		double ratio;
		double log_weight;

		if (!isfinite(filter->particles[index]) ||
		    !isfinite(filter->weights[index]) ||
		    filter->weights[index] < 0.0f)
			return -1;
		if (filter->weights[index] == 0.0f)
			continue;

		difference = (double)measurement - filter->particles[index];
		ratio = difference / filter->measurement_noise;
		log_weight = log(filter->weights[index]) -
			     0.5 * ratio * ratio;
		if (log_weight > maximum_log_weight)
			maximum_log_weight = log_weight;
	}

	if (maximum_log_weight == -DBL_MAX)
		return -1;

	for (index = 0; index < filter->particle_count; index++) {
		double difference;
		double ratio;
		double log_weight;
		double weight;

		if (filter->weights[index] == 0.0f) {
			filter->weights[index] = 0.0f;
			continue;
		}

		difference = (double)measurement - filter->particles[index];
		ratio = difference / filter->measurement_noise;
		log_weight = log(filter->weights[index]) -
			     0.5 * ratio * ratio;
		weight = exp(log_weight - maximum_log_weight);
		filter->weights[index] = (float)weight;
		total += weight;
	}

	if (!isfinite(total) || total <= 0.0)
		return -1;

	for (index = 0; index < filter->particle_count; index++) {
		float weight = (float)(filter->weights[index] / total);

		filter->weights[index] = weight;
		estimate += (double)filter->particles[index] * weight;
		normalized_total += weight;
		square_total += (double)weight * weight;
	}

	if (!isfinite(normalized_total) || normalized_total <= 0.0 ||
	    square_total <= 0.0)
		return -1;

	estimate /= normalized_total;
	if (!isfinite(estimate) || estimate > FLT_MAX ||
	    estimate < -FLT_MAX)
		return -1;

	filter->estimate = (float)estimate;
	if (normalized_total * normalized_total / square_total <
	    (double)filter->particle_count * 0.5)
		particle_filter_resample(filter);

	return 0;
}

float particle_filter_get_estimate(const struct particle_filter *filter)
{
	if (!particle_filter_is_valid(filter))
		return 0.0f;

	return filter->estimate;
}

float particle_filter_get_effective_count(
				const struct particle_filter *filter)
{
	double total = 0.0;
	double square_total = 0.0;
	double effective_count;
	size_t index;

	if (!particle_filter_is_valid(filter))
		return 0.0f;

	for (index = 0; index < filter->particle_count; index++) {
		if (!isfinite(filter->weights[index]) ||
		    filter->weights[index] < 0.0f)
			return 0.0f;

		total += filter->weights[index];
		square_total += filter->weights[index] * filter->weights[index];
	}

	if (total <= 0.0 || square_total <= 0.0)
		return 0.0f;

	effective_count = total * total / square_total;
	if (effective_count > filter->particle_count)
		effective_count = filter->particle_count;

	return (float)effective_count;
}
