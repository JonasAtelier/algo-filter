/*
 * Track a value that is moving, using measurements that are badly noisy.
 * Simulated, so it runs anywhere with no hardware.
 */
#include "particle_filter.h"

#include <math.h>
#include <stdio.h>

#define PARTICLES	200
#define STEPS		60
#define MOVEMENT	0.5f	/* how far the truth moves each step */
#define SENSOR_NOISE	2.0f	/* how badly the sensor lies, one sigma */

/* Deterministic rough noise, so the run is repeatable. */
static float jitter(unsigned long *seed, float scale)
{
	*seed = *seed * 1103515245UL + 12345UL;

	return ((float)((*seed >> 16) % 2001UL) / 1000.0f - 1.0f) * scale;
}

int main(void)
{
	static float particles[PARTICLES];
	static float weights[PARTICLES];
	static float scratch[PARTICLES];
	struct particle_filter filter;
	unsigned long seed = 7UL;
	float truth = 0.0f;
	float measured;
	float raw_error = 0.0f;
	float filtered_error = 0.0f;
	int step;

	if (particle_filter_init(&filter, particles, weights, scratch,
				 PARTICLES, 0.0f, 2.0f, 0.1f, SENSOR_NOISE,
				 1234U) != 0) {
		puts("init failed");
		return 1;
	}

	puts(" step    truth   measured   estimate   live particles");
	for (step = 1; step <= STEPS; step++) {
		truth += MOVEMENT;
		measured = truth + jitter(&seed, SENSOR_NOISE);

		particle_filter_predict(&filter, MOVEMENT);
		particle_filter_update(&filter, measured);

		raw_error += fabsf(measured - truth);
		filtered_error += fabsf(particle_filter_get_estimate(&filter) -
					truth);

		if (step % 10 == 0)
			printf("%5d %8.2f %10.2f %10.2f %14.1f\n", step,
			       (double)truth, (double)measured,
			       (double)particle_filter_get_estimate(&filter),
			       (double)particle_filter_get_effective_count(&filter));
	}

	printf("\naverage error - raw sensor %.2f, filtered %.2f\n",
	       (double)(raw_error / STEPS),
	       (double)(filtered_error / STEPS));
	puts("\"live particles\" is how many still carry useful weight; when it");
	puts("falls below half, the filter resamples and refills the cloud.");

	return 0;
}
