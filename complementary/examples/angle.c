/*
 * Simulated gyroscope and accelerometer, so this runs anywhere with no
 * hardware. The gyroscope drifts, the accelerometer is noisy, and the filter
 * gets a usable angle out of the pair.
 */
#include "complementary_filter.h"

#include <stdio.h>

#define DT_S		0.01f	/* 100 Hz */
#define RATE_DPS	10.0f	/* the board really is turning this fast */
#define GYRO_BIAS_DPS	2.0f	/* ... but the gyro thinks it is turning more */
#define STEPS		500

/* Deterministic +/- 1 degree jitter, so the run is repeatable. */
static float jitter(unsigned long *seed)
{
	*seed = *seed * 1103515245UL + 12345UL;

	return (float)((*seed >> 16) % 2000UL) / 1000.0f - 1.0f;
}

int main(void)
{
	struct complementary_filter filter;
	unsigned long seed = 1UL;
	float truth_deg = 0.0f;
	float gyro_only_deg = 0.0f;
	float filtered_deg = 0.0f;
	float accel_deg;
	int step;

	complementary_filter_init(&filter, 0.0f, 0.98f);

	puts(" time    truth   gyro only   accel only   filtered");
	for (step = 1; step <= STEPS; step++) {
		truth_deg += RATE_DPS * DT_S;
		accel_deg = truth_deg + jitter(&seed);
		gyro_only_deg += (RATE_DPS + GYRO_BIAS_DPS) * DT_S;

		filtered_deg = complementary_filter_update(&filter,
							   RATE_DPS + GYRO_BIAS_DPS,
							   accel_deg, DT_S);

		if (step % 100 == 0)
			printf("%5.2fs %8.2f %11.2f %12.2f %10.2f\n",
			       (double)((float)step * DT_S), (double)truth_deg,
			       (double)gyro_only_deg, (double)accel_deg,
			       (double)filtered_deg);
	}

	printf("\ngyro alone drifted %.2f degrees off; the filter is %.2f off.\n",
	       (double)(gyro_only_deg - truth_deg),
	       (double)(filtered_deg - truth_deg));
	puts("The accelerometer alone is never far off, but it is never steady.");

	return 0;
}
