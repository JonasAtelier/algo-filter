# ESP32 Particle Filter

A small, beginner-friendly Arduino ESP32 library for estimating one changing
value with a particle filter. It is written in C, performs no dynamic
allocation, and has no dependencies beyond the C math library.

## Installation

Download or clone this repository into your Arduino libraries directory, then
restart the Arduino IDE. Include the library with:

```cpp
#include <particle_filter.h>
```

## Minimal use

```cpp
static struct particle_filter filter;
static float particles[100];
static float weights[100];
static float scratch[100];

void setup()
{
	particle_filter_init(&filter, particles, weights, scratch, 100,
			     0.0f, 5.0f, 0.2f, 1.0f, 1234U);
}

void loop()
{
	float known_movement = 0.1f;
	float sensor_measurement = read_sensor();

	particle_filter_predict(&filter, known_movement);
	particle_filter_update(&filter, sensor_measurement);

	float estimate = particle_filter_get_estimate(&filter);
}
```

The three arrays are owned by your application, must not overlap, and must
remain valid while the filter is used. Their lengths must match
`particle_count`. With 100 particles, they use about 1.2 KiB in total.

The initial value, spread, movement, process noise, measurement noise, and
sensor readings must all use the same unit. Spreads and noise values are
standard deviations. `particle_filter_init()`, `particle_filter_reset()`,
`particle_filter_predict()`, and `particle_filter_update()` return `0` on
success and `-1` for invalid input.

Call predict before update for each new sensor sample. The filter automatically
normalizes weights and resamples when too few particles remain effective.

See the [complete Arduino example](examples/basic_filter/basic_filter.ino) and
the [algorithm and tuning guide](docs/algorithm.md).

## Host tests

Run the dependency-free C tests and C++ linkage check on Linux:

```sh
make -C test
make -C test clean
```

## License

MIT
