#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "madgwick_filter.h"

int main()
{
	madgwick_filter filter;
	float yaw_deg;
	int step;

	/* beta 0 is pure gyro integration: 1 rad/s for 1 s is 1 radian. */
	madgwick_filter_init(&filter, 0.0f);
	for (step = 0; step < 1000; step++)
		madgwick_filter_update_imu(&filter, 0.0f, 0.0f, 1.0f,
					   0.0f, 0.0f, 9.81f, 0.001f);

	madgwick_filter_euler(&filter, 0, 0, &yaw_deg);
	if (std::fabs(yaw_deg - 57.29578f) > 0.5f)
		return EXIT_FAILURE;

	std::puts("PASS: C++ linkage smoke test");
	return EXIT_SUCCESS;
}
