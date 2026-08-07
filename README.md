# ESP32 Complementary Filter

A small, allocation-free Arduino ESP32 library for estimating an angle from
gyroscope and accelerometer measurements. The core is plain C and has no
Arduino or sensor-library dependencies.

## Install

Copy this directory into your Arduino `libraries` directory, then restart the
Arduino IDE. Include the library with:

```cpp
#include <complementary_filter.h>
```

## Use

Create a filter, initialize it, and call `complementary_filter_update()` for
each sensor sample:

```cpp
struct complementary_filter filter;

complementary_filter_init(&filter, 0.0f, 0.98f);

float angle = complementary_filter_update(&filter, gyro_rate_dps,
					   accel_angle_deg, dt_s);
```

The arguments are:

- `initial_angle_deg`: starting angle in degrees. The first accelerometer angle
  is usually a good startup value.
- `alpha`: gyroscope weight, automatically clamped to `[0, 1]`. `0.98` is a
  useful starting point, but should be tuned for sensor noise and sample rate.
- `gyro_rate_dps`: angular velocity in degrees per second.
- `accel_angle_deg`: angle calculated from the accelerometer, in degrees.
- `dt_s`: elapsed time in seconds.

Call `complementary_filter_reset()` to set a new angle without changing
`alpha`. An update with `dt_s <= 0` is ignored and returns the current angle.
Gyroscope and accelerometer values must describe the same axis and use the
same positive direction.

All public inputs must be finite. Invalid update inputs leave the last finite
angle unchanged. A non-finite initial angle becomes `0`, a non-finite reset
angle is ignored, and a non-finite `alpha` uses the safe default `0.98`.
Because the fields are public, each update also repairs a non-finite stored
angle to `0` and revalidates `alpha`. Passing a null filter pointer is safe;
update returns `0.0f`.

Accelerometer correction follows the shortest angular path across the
`-180`/`180` boundary. Gyroscope integration is not wrapped, so continuous
motion remains intuitive and the returned angle is not restricted to a
single revolution.

See `examples/basic_angle/basic_angle.ino` for an Arduino example and
`docs/algorithm.md` for an explanation of the filter.

## Native tests

The core can be tested on Linux without Arduino:

```sh
make -C test
```

## License

MIT
