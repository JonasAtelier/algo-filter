# ESP32 Particle Filter

Estimates one changing value with a particle filter. Plain C, no dynamic
allocation, no dependencies beyond the C math library.

## Install

Clone into your Arduino libraries directory and restart the IDE.

## Use

```cpp
#include <particle_filter.h>

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
	particle_filter_predict(&filter, 0.1f);              // known movement
	particle_filter_update(&filter, read_sensor());      // measurement

	float estimate = particle_filter_get_estimate(&filter);
}
```

Call `predict` before `update` for each sample. Weights are normalised and the
filter resamples automatically when too few particles remain effective.

### `particle_filter_init()`

| Argument | Meaning |
|---|---|
| `particles`, `weights`, `scratch` | Caller-owned arrays |
| `particle_count` | Length of all three |
| `initial_value` | Starting estimate |
| `initial_spread` | Standard deviation of the starting cloud |
| `process_noise` | Standard deviation added per `predict` |
| `measurement_noise` | Standard deviation assumed of the sensor |
| `seed` | RNG seed |

The three arrays **must not overlap** and must stay valid for the filter's
lifetime. At 100 particles they cost about 1.2 KiB total.

Initial value, spread, movement, both noise figures and your sensor readings must
all use the **same unit**. Spreads and noise are standard deviations.

`init`, `reset`, `predict` and `update` return `0` on success, `-1` on invalid
input.

See the [Arduino example](examples/basic_filter/basic_filter.ino) and the
[algorithm and tuning guide](docs/algorithm.md).

## Tests

```sh
make -C test
```

## License

MIT
