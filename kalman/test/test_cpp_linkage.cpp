/* The header claims C++ can use it. Prove the extern "C" block works. */
#include "kalman_filter.h"

#include <cassert>

int main()
{
	struct kalman_filter filter;
	float out = 0.0f;

	assert(kalman_filter_init(&filter) == KALMAN_FILTER_OK);
	assert(kalman_filter_update(&filter, 1.5f, &out) == KALMAN_FILTER_OK);
	assert(out == 1.5f);
	return 0;
}
