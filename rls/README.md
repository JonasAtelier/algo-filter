# f_rls

<p align="center">
  <img alt="platform: portable C99" src="https://img.shields.io/badge/platform-portable%20C99-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: MIT" src="https://img.shields.io/badge/license-MIT-f2bd68?style=flat-square&labelColor=0d2238">
</p>

Learns the handful of numbers that describe your plant, from its own inputs and
outputs, one sample at a time — recursive least squares in portable C99.

- One header, one source file. Drop them into any C project.
- C99 with `libm` and nothing else. No allocation, no globals, no I/O.
- Up to 8 parameters, fixed-size state, no matrix library.

## What it is for

You have a log of what you did and what happened. Somewhere in it is a number
you need and nobody wrote down.

- **`b0` for a controller.** `adrc` asks for one piece of plant
  knowledge: how much the measurement's rate changes per unit of output. Log a
  couple of seconds of command and speed, and this reads it off. See
  `examples/motor.c`.
- **A motor's friction.** Fit the speed change against current *and* speed at
  once and the drag term falls out beside the gain — the number that quietly
  changes as the machine warms up or the bearings age.
- **A sensor's scale and offset.** Against a reference, fit
  `reading = scale * truth + offset`. Two parameters, and with `lambda` below 1
  they keep following a drifting offset instead of averaging it away.

The common thread: the thing you want is *unknown*, but the model is **linear in
it**. That is the whole requirement, and it is a much weaker one than it sounds.

## Try it

```sh
make check    # run the tests, including the C++ linkage smoke test
make run      # identify a simulated DC motor from 2 s of current and speed
```

```c
#include "rls_filter.h"

struct rls_filter filter;
float phi[2];

rls_filter_init(&filter, 2, 1.0f, 1e4f);   /* 2 parameters, no forgetting */

/* once per sample */
phi[0] = u;
phi[1] = w;
rls_filter_update(&filter, phi, speed_change, NULL);

/* read it whenever you like */
float b0 = filter.theta[0] / dt;
```

Copy `include/rls_filter.h` and `src/rls_filter.c` into your project and build
them with your own sources. That is the whole install.

With CMake, link the target `algo::rls` instead; for ESP-IDF, use this folder
as a component. See [Use one library](../README.md#use-one-library).

## Writing the regressor

Everything here is one equation:

```
y = phi . theta
```

- **`theta`** is what you want to know. The filter owns it and updates it.
- **`y`** is the one number you measured this sample.
- **`phi`** is the list of measured things that `y` is built from, in the same
  order as `theta`. You build it fresh every sample.

Turning a model into those three is the only work. Take the motor: between two
samples `dt` apart, its speed moves by

```
w[k+1] - w[k] = dt * (b0 * u[k] - a * w[k])
```

You want `b0` and `a`. Everything else in that line — `dt`, the current `u`, the
speed `w` — you already have. So put the speed change on the left and the two
measured quantities it is made of on the right:

| | is |
|---|---|
| `y` | `w[k+1] - w[k]`, the speed change you just saw |
| `phi[0]` | `u[k]`, the current you commanded |
| `phi[1]` | `w[k]`, the speed you were at |
| `theta[0]` | comes out as `dt * b0` |
| `theta[1]` | comes out as `-dt * a` |

The `dt` falls out because it multiplies both terms; divide it back out when you
read `theta`. That is typical — **the parameters you recover are whatever
multiplies your regressor**, not necessarily the constants you named. Write the
equation down and read off the pairing.

Two rules and no others:

- `phi` must not contain the unknown. `u * b0` in a regressor is not a
  regressor; that is what makes a model nonlinear in its parameters.
- The entries of `phi` must actually move independently. If `u` is constant, `u`
  and `1` are the same column and neither parameter can be told from the other.

## The one knob

`lambda` is how long a sample keeps counting. Old data fades by a factor of
`lambda` per sample, so the filter effectively remembers the last
`1 / (1 - lambda)` of them.

| lambda | Remembers | Use it for |
|---|---|---|
| `1.0` | Everything, forever | A constant you are identifying once. The default choice |
| `0.99` | ~100 samples | A parameter that drifts slowly — temperature, wear |
| `0.95` | ~20 samples | A parameter that really changes, and clean data |
| `< 0.9` | Almost nothing | Rarely right. Noise gets tracked as if it were signal |

Below `1.0` there is no free lunch: the same forgetting that lets the estimate
follow a change also lets it wander when the input goes quiet, because old
certainty decays whether or not anything new arrives. Start at `1.0` and only
lower it if the thing you are estimating is genuinely moving.

The other number, `p0`, is just how loudly the filter admits it starts out
ignorant. Anything from `1e3` to `1e6` is fine and it stops mattering after a
few samples.

## The functions

| Function | What it does |
|---|---|
| `rls_filter_init` | `n` parameters, a `lambda` in `(0, 1]` and a `p0 > 0`. Starts at `theta = 0` |
| `rls_filter_update` | One sample: `phi` and the `y` it produced. Optionally hands back the error it made before learning from it |
| `rls_filter_predict` | What the estimate says `y` should be for a given `phi`. `NAN` if the filter is not usable |
| `rls_filter_reset` | Back to knowing nothing, keeping `n` and `lambda` |

Read `filter.theta` whenever you want; it is always the current best fit. The
error handed back by `rls_filter_update` is worth watching too — while it is
still large the estimate has not settled, and if it suddenly grows again the
plant has changed underneath you.

### Status codes

| Status | Meaning |
|---|---|
| `RLS_FILTER_OK` | Succeeded |
| `RLS_FILTER_ERR_NULL` | A required pointer was null |
| `RLS_FILTER_ERR_NOT_FINITE` | A supplied value was not finite |
| `RLS_FILTER_ERR_INVALID_VALUE` | Out of range: `n`, `lambda` or `p0` |
| `RLS_FILTER_ERR_NOT_INITIALIZED` | Not initialised, or the struct has been left invalid |
| `RLS_FILTER_ERR_NUMERIC` | The step produced no valid finite result |

A call that returns anything but `OK` leaves the filter byte-for-byte as it
found it, so one bad sample cannot poison an estimate you have spent a minute
building. C++ can use the header through its `extern "C"`, which `make check`
verifies. Separate instances are independent; synchronise a shared one yourself.

## What it does not do

- **No nonlinear models.** Linear in the *parameters* only. `y = a * exp(b * x)`
  is not this tool's problem unless you can take a log and make it one.
- **No correction for noise on the regressor.** Least squares assumes `phi` is
  known exactly and all the error is on `y`. When a noisy measurement appears in
  `phi` as well — the motor's own speed, above — the estimate is biased toward
  zero, and no amount of extra data removes it. Filter the input, or reach for
  instrumental variables.
- **No excitation.** This is the one that catches people. If your input never
  moves, the data contains no information about the parameters and nothing is
  learned. Worse, with `lambda < 1` a quiet input lets the covariance grow until
  the next sample of noise is treated as revelation. Wiggle the plant on purpose
  while you identify it, and freeze the estimate when you cannot.
- **No model selection, no delay estimation, no order detection.** You choose
  what goes in `phi`. Get that wrong and the filter will fit the wrong model
  beautifully.
- **No clock and no `dt`.** Sample spacing is yours to keep steady and yours to
  divide back out.

## License

MIT. See `LICENSE`.
