#include "kalman_filter.h"

#include <stdio.h>

int main(void)
{
	const float measurements[] = {
		20.0f, 24.0f, 18.0f, 23.0f, 19.0f, 21.0f,
	};
	struct kalman_filter filter;
	size_t i;

	if (kalman_filter_init_with_noise(&filter, 0.05f, 4.0f) !=
	    KALMAN_FILTER_OK)
		return 1;

	puts("raw,filtered");
	for (i = 0; i < sizeof(measurements) / sizeof(measurements[0]); i++) {
		float filtered;

		if (kalman_filter_update(&filter, measurements[i],
					 &filtered) != KALMAN_FILTER_OK)
			return 1;

		printf("%.2f,%.2f\n", measurements[i], filtered);
	}

	return 0;
}
