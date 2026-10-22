#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "complementary_filter.h"

int main()
{
	complementary_filter filter;
	float angle_deg;

	complementary_filter_init(&filter, 10.0f, 1.0f);
	angle_deg = complementary_filter_update(&filter, 5.0f, 10.0f, 2.0f);
	if (std::fabs(angle_deg - 20.0f) > 0.0001f)
		return EXIT_FAILURE;

	std::puts("PASS: C++ linkage smoke test");
	return EXIT_SUCCESS;
}
