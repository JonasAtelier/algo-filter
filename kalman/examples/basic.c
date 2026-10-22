#include "kalman_filter.h"

#include <stdio.h>

int main(void)
{
	const float measurements[] = {
		10.0f, 11.2f, 9.4f, 10.8f, 9.7f, 10.3f,
	};
	struct kalman_filter filter;
	size_t i;

	if (kalman_filter_init(&filter) != KALMAN_FILTER_OK)
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
