#include "kalman_filter.h"

int main()
{
	struct kalman_filter filter;
	float result;

	if (kalman_filter_init(&filter) != KALMAN_FILTER_OK)
		return 1;

	if (kalman_filter_update(&filter, 10.0f, &result) !=
	    KALMAN_FILTER_OK)
		return 1;

	return result == 10.0f ? 0 : 1;
}
