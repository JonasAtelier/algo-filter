# ESP32 Complementary Filter

Estimates an angle from gyroscope and accelerometer measurements. Allocation-free,
plain C core with no Arduino or sensor-library dependencies.

## Install

Copy this directory into your Arduino `libraries` directory and restart the IDE.

## Use

```cpp
#include <complementary_filter.h>

struct complementary_filter filter;

complementary_filter_init(&filter, 0.0f, 0.98f);

float angle = complementary_filter_update(&filter, gyro_rate_dps,
                                          accel_angle_deg, dt_s);
```

| Argument | Meaning |
|---|---|
| `initial_angle_deg` | Starting angle. The first accelerometer angle is a good value |
| `alpha` | Gyroscope weight, clamped to `[0, 1]`. `0.98` is a starting point — tune for your noise and sample rate |
| `gyro_rate_dps` | Angular velocity, degrees per second |
| `accel_angle_deg` | Angle from the accelerometer, degrees |
| `dt_s` | Elapsed time, seconds |

`complementary_filter_reset()` sets a new angle without touching `alpha`.

Gyro and accelerometer values must describe the **same axis with the same
positive direction**.

### Invalid input

All inputs must be finite; the filter degrades rather than faults.

- `dt_s <= 0` — ignored, returns the current angle
- Invalid update input — last finite angle kept
- Non-finite initial angle — becomes `0`
- Non-finite reset angle — ignored
- Non-finite `alpha` — falls back to `0.98`
- Null filter pointer — safe, update returns `0.0f`

The fields are public, so each update also repairs a non-finite stored angle to
`0` and revalidates `alpha`.

Accelerometer correction takes the shortest path across the `-180`/`180`
boundary. Gyroscope integration is **not** wrapped, so continuous motion stays
intuitive and the angle is not confined to one revolution.

See `examples/basic_angle/basic_angle.ino`, and `docs/algorithm.md` for how the
filter works.

## Tests

Runs on Linux without Arduino:

```sh
make -C test
```

## License

MIT
