# Portable Scalar Kalman Filter

This library smooths one stream of noisy numeric measurements while continuing
to follow real changes. It implements a one-dimensional random-walk Kalman
filter in portable C, uses no dynamic memory, and keeps all state in an object
owned by the caller.

Use it for one independent value such as temperature, distance, pressure, an
ADC reading, or one sensor axis. It is not a matrix filter and does not combine
multiple sensor axes.

## What the filter does

For each measurement, the filter keeps an estimate and an uncertainty. The
first update seeds the estimate with the measurement. Later updates smooth the
stream while allowing the estimate to move as the measured value changes.

## When to use it

Use one filter instance for each independent scalar signal. It is a good fit
when individual samples are noisy but the underlying value changes gradually.

## Visual intuition

```text
noisy measurement ──┐
                    ├─> predict uncertainty ─> calculate gain ─> new estimate
previous estimate ──┘                                      │
previous uncertainty ──────────────────────────────────────┘
```

The estimate moves between the previous estimate and the new measurement. For
the default tuning, the first four rows from `examples/basic.c` are:

| Raw | Filtered |
| ---: | ---: |
| 10.00 | 10.00 |
| 11.20 | 10.60 |
| 9.40 | 10.20 |
| 10.80 | 10.35 |

## Quick start

```c
#include "kalman_filter.h"

#include <stdio.h>

int main(void)
{
	struct kalman_filter filter;
	float filtered;

	if (kalman_filter_init(&filter) != KALMAN_FILTER_OK)
		return 1;

	if (kalman_filter_update(&filter, 10.0f, &filtered) !=
	    KALMAN_FILTER_OK)
		return 1;

	printf("%.2f\n", filtered);
	return 0;
}
```

## Build the examples

Build the default-tuning example:

```sh
cc -std=c99 -Wall -Wextra -Werror -pedantic -Isrc \
  examples/basic.c src/kalman_filter.c -lm -o basic
```

Build the custom-tuning example:

```sh
cc -std=c99 -Wall -Wextra -Werror -pedantic -Isrc \
  examples/custom_tuning.c src/kalman_filter.c -lm -o custom_tuning
```

## Tuning

`kalman_filter_init()` uses process noise `0.01f` and measurement noise
`1.0f`. Process noise is finite and `>= 0`; increasing it follows real change
faster. Measurement noise is finite and `> 0`; increasing it smooths
measurements more. Use `kalman_filter_init_with_noise()` to choose both values
when initializing an instance.

## API

```c
enum kalman_filter_status
kalman_filter_init(struct kalman_filter *filter);

enum kalman_filter_status
kalman_filter_init_with_noise(struct kalman_filter *filter,
                              float process_noise,
                              float measurement_noise);

enum kalman_filter_status
kalman_filter_reset(struct kalman_filter *filter);

enum kalman_filter_status
kalman_filter_update(struct kalman_filter *filter,
                     float measurement,
                     float *result);
```

`kalman_filter_reset()` discards the estimate so that the next update seeds a
new estimate, while preserving the configured tuning.

## Error handling

Every public function returns one of these status values:

| Status | Meaning |
| --- | --- |
| `KALMAN_FILTER_OK` | The operation succeeded. |
| `KALMAN_FILTER_ERR_NULL` | A required pointer was null. |
| `KALMAN_FILTER_ERR_NOT_FINITE` | A supplied numeric value was not finite. |
| `KALMAN_FILTER_ERR_INVALID_VALUE` | A supplied value was outside its valid range, including invalid tuning or an output pointer into the filter object. |
| `KALMAN_FILTER_ERR_NOT_INITIALIZED` | The filter was not initialized or has invalid state. |
| `KALMAN_FILTER_ERR_NUMERIC` | A calculation could not produce a valid finite result. |

Callers must initialize a filter before updating or resetting it. A failed call
preserves both the filter and the output. The output pointer passed to
`kalman_filter_update()` must not point into the filter object.

## Portability and concurrency

The core requires C99 and standard headers, and commonly links with `-lm`. C++
code can use it through the header's `extern "C"` declaration. The library has
no heap, platform, or global mutable state. Separate instances are independent;
the caller is responsible for synchronizing access to a shared instance.

## Algorithm

This is a scalar random-walk Kalman filter. Its equations are:

```text
predicted covariance = covariance + process noise
gain = predicted covariance / (predicted covariance + measurement noise)
estimate = previous estimate + gain * (measurement - previous estimate)
covariance = (1 - gain) * (1 - gain) * predicted covariance
             + gain * gain * measurement noise
```

The final covariance equation is the Joseph form.
