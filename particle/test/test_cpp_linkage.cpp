#include "particle_filter.h"

int main()
{
	struct particle_filter filter;
	float particles[2];
	float weights[2];
	float scratch[2];

	return particle_filter_init(&filter, particles, weights, scratch, 2U,
				    0.0f, 0.0f, 0.0f, 1.0f, 1U);
}
