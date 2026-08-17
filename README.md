# ESP32 Kalman Filter

ESP32 Kalman Filter is a small Arduino library for smoothing one stream of
noisy sensor readings. Create a filter and call `kalman_filter_update()` for
every sample. No matrix math, dynamic memory, or signal-processing background
is required.

It works well for values such as distance, temperature, pressure, ADC readings,
or one accelerometer axis. The library is written in C, follows Linux kernel
style, and can be called directly from Arduino C++ sketches.

## Quick start

```cpp
#include <kalman_filter.h>

struct kalman_filter filter;

void setup()
{
	Serial.begin(115200);
	kalman_filter_init(&filter);
}

void loop()
{
	float filtered;
	float raw = analogRead(34);

	if (kalman_filter_update(&filter, raw, &filtered) ==
	    KALMAN_FILTER_OK)
		Serial.println(filtered);
}
```

The first measurement becomes the starting value. Later measurements are
smoothed automatically.

## Installation

Copy this directory into the Arduino sketchbook `libraries` directory, then
restart the Arduino IDE:

```text
Arduino/
└── libraries/
    └── esp32-kalman-filter/
```

The repository can also be downloaded from your private Git host as a ZIP and
installed with **Sketch > Include Library > Add .ZIP Library**.

## Tuning

The defaults are a good starting point:

```c
kalman_filter_init(&filter);
```

For custom smoothing, set the process and measurement noise:

```c
kalman_filter_init_with_noise(&filter, 0.05f, 4.0f);
```

| Setting | Increase it to |
| --- | --- |
| Process noise | Follow real changes faster |
| Measurement noise | Smooth noisy measurements more |

Process noise must be zero or greater. Measurement noise must be greater than
zero. Both values must be finite.

## API

| Function | Purpose |
| --- | --- |
| `kalman_filter_init()` | Start a filter with beginner-friendly defaults |
| `kalman_filter_init_with_noise()` | Start a filter with custom noise values |
| `kalman_filter_update()` | Add a measurement and receive its filtered value |
| `kalman_filter_set_noise()` | Change tuning without losing the estimate |
| `kalman_filter_reset()` | Make the next measurement a new starting value |
| `kalman_filter_value()` | Read the current filtered value |

Every function returns `KALMAN_FILTER_OK` on success. Null pointers, invalid
values, or numeric operations return a negative `kalman_filter_status` value.
A failed operation does not change the filter or output value.

Always call an initialization function before using a filter. For static or
global filters that need an explicit empty value, use
`KALMAN_FILTER_INITIALIZER`. Store output in a separate `float`; do not use a
member of the filter structure as the output variable.

## How it works

This library implements a one-dimensional random-walk Kalman filter. It tracks
one estimate and its uncertainty:

```text
predicted covariance = covariance + process noise
gain                 = predicted covariance
                       / (predicted covariance + measurement noise)
estimate             = estimate + gain * (measurement - estimate)
```

The covariance update uses the numerically stable Joseph form.

## Run host tests

```sh
make -f tests/Makefile test
```

The host tests require a C11 compiler and a C++17 compiler. Hardware behavior
must still be tested on the target ESP32 board.
