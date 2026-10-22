/*
 * Identify a DC motor from a log of its current and its speed.
 *
 * Between two samples dt apart, the speed of a motor with viscous drag moves by
 *
 *   w[k+1] - w[k] = dt * (b0 * u[k] - a * w[k])
 *
 * which is linear in b0 and a. Take the speed change as the output y and the
 * pair (u, w) as the regressor, and RLS hands back dt * b0 and -dt * a.
 *
 * Here the log comes from a simulated motor so the answer can be checked, but
 * nothing below knows that; feed it two columns from a real machine instead.
 */
#include "rls_filter.h"

#include <stdio.h>

#define RATE_HZ		1000
#define DT		(1.0f / (float)RATE_HZ)
#define SAMPLES		(2 * RATE_HZ)	/* 2 seconds */

/* The motor we are pretending not to know. */
#define TRUE_B0		250.0f		/* (rad/s) per second, per amp */
#define TRUE_A		5.0f		/* drag, 1/s */

int main(void)
{
	struct rls_filter filter;
	float w = 0.0f;
	int k;

	/* Two parameters, lambda 1 because they are constants worth
	 * remembering, and a p0 that says we know nothing yet. */
	if (rls_filter_init(&filter, 2, 1.0f, 1e4f) != RLS_FILTER_OK)
		return 1;

	for (k = 0; k < SAMPLES; k++) {
		/* A current that steps every 0.25 s. Standing still teaches
		 * the filter nothing: the steps are the excitation. */
		float u = (k / 250) % 2 ? 3.0f : 1.0f;
		float dw = DT * (TRUE_B0 * u - TRUE_A * w);
		float phi[2];

		phi[0] = u;
		phi[1] = w;

		if (rls_filter_update(&filter, phi, dw, NULL) != RLS_FILTER_OK)
			return 1;

		w += dw;
	}

	printf("%d samples at %d Hz, %.1f s of motor\n\n",
	       SAMPLES, RATE_HZ, (double)(SAMPLES * DT));
	printf("            identified        true\n");
	printf("  b0   %14.3f  %10.3f\n",
	       (double)filter.theta[0] / (double)DT, (double)TRUE_B0);
	printf("  a    %14.3f  %10.3f\n\n",
	       -(double)filter.theta[1] / (double)DT, (double)TRUE_A);

	puts("(the simulated log is noiseless, so the fit is exact;");
	puts(" a real one will not be)\n");

	/* b0 is the one number adrc asks you for. */
	printf("adrc_cfg.b0 = %.1f\n",
	       (double)filter.theta[0] / (double)DT);

	return 0;
}
