/*
 * A simulated IMU, so this runs anywhere with no hardware. The sensor starts
 * level and rolls to 30 degrees over one second. The gyro and accelerometer
 * readings are generated from the true angle, then handed to the filter, which
 * never sees the true angle itself.
 */
#include "madgwick_filter.h"

#include <math.h>
#include <stdio.h>

#define DT_S		0.01f	/* 100 Hz */
#define ROLL_RATE_DPS	30.0f	/* one second of this reaches 30 degrees */
#define STEPS		100
#define PRINT_EVERY	10	/* every 100 ms */
#define GRAVITY		9.81f
#define DEG_TO_RAD	0.017453292519943295f

int main(void)
{
	struct madgwick_filter filter;
	float truth_deg = 0.0f;
	float estimate_deg;
	float half_rad;
	float q0;
	float q1;
	int step;

	madgwick_filter_init(&filter, 0.1f);

	puts(" time   true roll   estimated   error");
	for (step = 1; step <= STEPS; step++) {
		truth_deg += ROLL_RATE_DPS * DT_S;

		/*
		 * Gravity as a sensor at this roll angle reads it: the earth z
		 * axis rotated into the sensor frame by (cos(r/2), sin(r/2),
		 * 0, 0).
		 */
		half_rad = truth_deg * DEG_TO_RAD * 0.5f;
		q0 = cosf(half_rad);
		q1 = sinf(half_rad);

		madgwick_filter_update_imu(&filter,
					   ROLL_RATE_DPS * DEG_TO_RAD,
					   0.0f, 0.0f,
					   0.0f,
					   GRAVITY * 2.0f * q0 * q1,
					   GRAVITY * (1.0f - 2.0f * q1 * q1),
					   DT_S);

		if (step % PRINT_EVERY == 0) {
			madgwick_filter_euler(&filter, &estimate_deg, NULL,
					      NULL);
			printf("%5.2fs %10.2f %11.2f %7.2f\n",
			       (double)((float)step * DT_S),
			       (double)truth_deg, (double)estimate_deg,
			       (double)(estimate_deg - truth_deg));
		}
	}

	puts("\nThe filter is given no starting angle and no true angle, only");
	puts("a turn rate and a direction of gravity, one sample at a time.");

	return 0;
}
