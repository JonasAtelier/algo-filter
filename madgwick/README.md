# f_madgwick

<p align="center">
  <img alt="platform: portable C99" src="https://img.shields.io/badge/platform-portable%20C99-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: MIT" src="https://img.shields.io/badge/license-MIT-f2bd68?style=flat-square&labelColor=0d2238">
</p>

Turns a gyroscope and an accelerometer, optionally with a magnetometer, into a
full orientation — Madgwick's filter in portable C99.

- One header, one source file. Drop them into any C project.
- C99 with `libm` and nothing else. No allocation, no globals, no I/O.
- Nothing platform-specific — microcontroller, Linux, anywhere.

## When to use this instead of f_complementary

A complementary filter tracks **one angle**. That is the right tool for a
balancing robot leaning in one direction, and this library is overkill for it.

Reach for this one when the answer is an orientation rather than an angle.

- **All three axes at once.** Roll and pitch are not independent once the
  device really moves; tilting in one changes what the other means. This filter
  fuses them together instead of running two filters that quietly disagree.
- **Yaw.** An accelerometer cannot see heading — gravity looks the same whichever
  way you face. Add a magnetometer and this filter gives you yaw too, corrected
  against north instead of drifting away with the gyro.
- **No gimbal trouble.** The state is a quaternion, so pointing straight up or
  straight down is an ordinary orientation, not a place where the maths falls
  apart. Angles are only computed when you ask for them.

## Try it

```sh
make check    # run the tests, including the C++ linkage smoke test
make run      # roll a simulated sensor to 30 degrees and watch it tracked
```

```c
#include "madgwick_filter.h"

struct madgwick_filter filter;
float roll_deg, pitch_deg, yaw_deg;

madgwick_filter_init(&filter, 0.1f);

/* once per sample */
madgwick_filter_update_imu(&filter, gx, gy, gz, ax, ay, az, dt_s);

madgwick_filter_euler(&filter, &roll_deg, &pitch_deg, &yaw_deg);
```

Copy `include/madgwick_filter.h` and `src/madgwick_filter.c` into your project
and build them with your own sources. That is the whole install.

## The one knob

`beta` is how hard the accelerometer and magnetometer are allowed to pull the
gyro back toward the truth.

| beta | Behaviour |
|---|---|
| `0` | Gyro only. Perfectly smooth, and it drifts away for good |
| `0.1` | The default, and a good place to start |
| Higher | Settles faster after a bump, jitters more while still |

Because the correction step is normalised, it moves a fixed amount per sample
no matter how small the error is. A resting filter therefore hovers near the
truth rather than settling exactly onto it, and a larger `beta` widens that
hover. Start at `0.1`, raise it if the filter takes too long to catch up,
lower it if it will not sit still.

## Frames and units

Stated once, and assumed everywhere.

| | Convention |
|---|---|
| Sensor frame | `x` forward, `y` left, `z` up |
| Earth frame | `x` north (magnetic), `y` west, `z` up |
| Quaternion | `q = (w, x, y, z)`, earth frame to sensor frame, unit length |
| Gyroscope | Radians per second, right-hand positive about each sensor axis |
| Accelerometer | Any unit — only the direction is used. Level and still reads `+1 g` on `z` |
| Magnetometer | Any unit — only the direction is used |
| `dt_s` | Seconds since the previous sample |
| Angles out | Degrees. Roll about `x`, pitch about `y`, yaw about `z`. Yaw `0` faces north |

Both frames are right-handed. Your sensor almost certainly does not use this
convention out of the box; permute and negate its axes on the way in.

## The functions

| Function | What it does |
|---|---|
| `madgwick_filter_init` | Start level with a chosen `beta`. A non-finite or negative `beta` uses `0.1` |
| `madgwick_filter_reset` | Jump to a known orientation. `NULL` means level; the quaternion is normalised for you |
| `madgwick_filter_update_imu` | One gyro and accelerometer sample. Roll and pitch corrected, yaw left to the gyro |
| `madgwick_filter_update` | Same plus a magnetometer, which corrects yaw as well |
| `madgwick_filter_euler` | Read out roll, pitch and yaw in degrees. Any pointer may be `NULL` |

### Invalid input

The filter degrades rather than faults.

- Non-finite gyro, accelerometer, magnetometer or `dt_s` — the step is skipped
  and the orientation is unchanged
- `dt_s <= 0` — same
- A zero accelerometer vector — free fall, so there is no down to correct
  against; the step integrates the gyro only
- A zero magnetometer vector — falls back to `madgwick_filter_update_imu`
- A null filter pointer — safe, and does nothing

The fields are public, so each update also repairs a non-finite or zero
quaternion back to level and revalidates `beta`.

## What it does not do

- **No gyro bias estimation.** A constant gyro offset is fought by the
  correction term rather than learned and removed, so it leaves a small steady
  error. Remove the bias yourself while the device is still.
- **No sensor calibration.** Scale, offset and axis misalignment are yours to
  fix. A magnetometer in particular needs hard and soft iron calibration before
  its heading is worth anything.
- **No unit conversion or axis remapping.** Feed it the convention in the table
  above.
- **No sample timing.** You supply `dt_s`, so a jittery loop is a jittery
  filter.

Feed it calibrated data and it will give you a good orientation. Feed it raw
sensor output and it will faithfully track a wrong one.

## License

MIT
