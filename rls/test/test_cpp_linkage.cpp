/* The header claims C++ can use it. Prove the extern "C" block works. */
#include "rls_filter.h"

#include <cassert>
#include <cmath>

int main()
{
	struct rls_filter filter;
	const float phi[2] = { 2.0f, 1.0f };
	float error = 0.0f;

	assert(rls_filter_init(&filter, 2, 1.0f, 1e4f) == RLS_FILTER_OK);
	assert(rls_filter_update(&filter, phi, 5.0f, &error) == RLS_FILTER_OK);
	assert(error == 5.0f);		/* theta started at zero */
	assert(std::fabs(rls_filter_predict(&filter, phi) - 5.0f) < 1e-3f);
	assert(rls_filter_reset(&filter, 1e4f) == RLS_FILTER_OK);
	return 0;
}
