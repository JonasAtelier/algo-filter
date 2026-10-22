# f_ekf

<p align="center">
  <img alt="platform: portable C99" src="https://img.shields.io/badge/platform-portable%20C99-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: MIT" src="https://img.shields.io/badge/license-MIT-f2bd68?style=flat-square&labelColor=0d2238">
</p>

Estimates several coupled states at once from a sensor that does not measure
them directly — an extended Kalman filter for up to 8 states and 4
measurements, in portable C99.

- One header, one source file. Drop them into any C project.
- C99 with `libm` and nothing else. No allocation, no globals, no I/O.
- Nothing platform-specific — microcontroller, Linux, anywhere.

## When f_kalman is not enough

f_kalman smooths **one number** that wanders on its own. That is the right tool
for a temperature or a single ADC channel, and this library is overkill for it.

Reach for this one when any of these is true.

- **Your states are coupled.** Position and velocity are not two independent
  signals; each is evidence about the other. Two scalar filters cannot know
  that, so they drift apart. This one carries the whole covariance, so a
  position measurement sharpens the velocity too.
- **Something you care about is never measured.** Feed this filter only a noisy
  angle and it will still hand you the rate, because the model says how one
  follows from the other. `examples/pendulum.c` does exactly that.
- **Your sensor is nonlinear.** Range and bearing, a load cell curve, an
  inverse-square field: the measurement is some function of the state rather
  than the state itself. Write that function and the filter linearises it for
  you, each step, around wherever the estimate currently is.

## Try it

```sh
make check    # run the tests, including the C++ linkage smoke test
make run      # a pendulum's rate from a noisy angle alone, then a heading
              # from a gyro, a spiky compass and a second sensor
```

```
 time   true rate   estimated   error
 0.10s      -0.464      -0.710  -0.246
 0.50s      -1.549      -1.521   0.028
 1.00s      -0.089      -0.098  -0.009
 2.00s       0.177       0.178   0.001
```

Copy `include/ekf_filter.h` and `src/ekf_filter.c` into your project and build
them with your own sources. That is the whole install.

With CMake, link the target `algo::ekf` instead; for ESP-IDF, use this folder
as a component. See [Use one library](../README.md#use-one-library).

## Describing your system

You supply two functions. The filter supplies everything else.

- **`f`** — where the state goes next. `x_next = f(x, u, dt)`. `u` is your
  control input, or `NULL` if the plant has none; it is passed through
  untouched, so its size and meaning are yours.
- **`h`** — what the sensor would read if the state were `x`. `z = h(x)`.

Both are plain C functions taking the estimate as a `float` array. Neither ever
sees the covariance, and neither is allowed to fail.

```c
#include "ekf_filter.h"

/* Two states: position and velocity. One measurement: position. */
static void model(const float *x, const float *u, float dt, float *xn, void *ctx)
{
	(void)u; (void)ctx;
	xn[0] = x[0] + x[1] * dt;
	xn[1] = x[1];
}

static void sensor(const float *x, float *z, void *ctx)
{
	(void)ctx;
	z[0] = x[0];
}

struct ekf_filter filter;
const float x0[2] = { 0.0f, 0.0f };
const float P0[4] = { 10.0f, 0.0f, 0.0f, 10.0f };
const float Q[4]  = { 1e-6f, 0.0f, 0.0f, 1e-6f };
const float R[1]  = { 0.25f };

ekf_filter_init(&filter, 2, 1, x0, P0, Q, R);
filter.f = model;
filter.h = sensor;

/* once per sample */
ekf_filter_predict(&filter, NULL, dt);
ekf_filter_update(&filter, z, NULL);

printf("%.3f at %.3f/s\n", filter.x[0], filter.x[1]);
```

Every matrix here is row-major and packed to its real size: an `n` state filter
uses the first `n * n` entries of `P` and `Q`, and an `m` measurement filter the
first `m * m` of `R`. `ctx` is handed to every callback, which is where the
things your model needs but the state does not hold — a mass, a gear ratio, the
observer's own position — belong.

### Jacobians are optional

The filter needs `df/dx` and `dh/dx` at the current estimate. Leave `F` and `H`
`NULL` and it works them out by central differences, calling your `f` and `h`
twice per state. That is `2n` extra calls per step, and for anything short of a
tight control loop it is free. Set them when the derivative is easy to write, or
when your `f` is expensive.

`jacobian_step` sets the perturbation, and `0` selects `1e-4`. The step is
scaled by the size of each state so it survives being stored back into a
`float`, and the divisor is the difference the two perturbed floats actually
hold rather than the step that was asked for.

Two things to watch in a nonlinear model. `h` must be continuous near the
estimate, so an angle that wraps at `+/- pi` needs the help below — left alone,
the filter subtracts `h(x)` from `z` literally and does not know an angle from
a distance. And the linearisation is only good near the current estimate, so a
badly wrong `x0` with a confident `P0` can settle on the wrong answer.

### Angles

Two optional callbacks, both `NULL` after `init`.

- **`residual`** — `y = z - h(x)`, your way. A heading of `-3.1` measured
  against a predicted `+3.1` is `0.08`, not `-6.2`; wrap it here and the filter
  never sees the jump. It also decides what `innovation` hands back.
- **`normalize`** — put a fresh state back in range. It runs on the new `x`
  after every predict and every successful update, before it is stored, and
  gets the filter's `ctx`.

```c
static void wrapped(const float *z, const float *predicted, float *y, void *ctx)
{
	(void)ctx;
	y[0] = remainderf(z[0] - predicted[0], 6.2831853f);	/* into [-pi, pi] */
}

filter.residual = wrapped;
filter.normalize = wrap_heading;	/* x[0] into (-pi, pi] */
```

`examples/heading.c` does both, across `+/- pi` several times.

## More than one sensor

The filter's own `m`, `h`, `H`, `R` and `ctx` are one sensor. Describe any other
in a `struct ekf_sensor` — its own size, model, noise, residual, gate and `ctx` —
and hand it to `ekf_filter_update_sensor`. Each sensor updates when its reading
arrives; nothing has to be in step.

```c
struct ekf_sensor gps = { 0 };

gps.m = 2;
gps.h = gps_position;		/* gps.H NULL: numeric, filter.jacobian_step */
gps.R[0] = 4.0f;
gps.R[3] = 4.0f;
gps.gate = 9.21f;

ekf_filter_update_sensor(&filter, &gps, z, NULL);
```

The sensor is checked on every call: `m` from 1 to 4, `h` set, `R` finite and
symmetric within `1e-6`, `gate` finite and not negative. Run with the filter's
own values it gives exactly what `ekf_filter_update` gives.

## Throwing out outliers

Every update works out the normalised innovation squared, `nis = y' S^-1 y`
with `S = H P H' + R` — how surprising this reading is, in units of its own
expected spread. It falls out of the solve the update does anyway, and is left
in `filter.nis` after every update that gets that far. A value too large for a
`float` is stored as infinity, which any gate rejects.

Set `gate` (on the filter for `ekf_filter_update`, on the sensor for
`ekf_filter_update_sensor`) and a reading with `nis > gate` is refused:
`EKF_FILTER_REJECTED`, `x` and `P` untouched, `nis` and `innovation` still
written so you can log what was thrown out. `0` turns the gate off.

The threshold is a chi-square quantile for `m` degrees of freedom: `9` is about
3 sigma for one value, `9.21` keeps 99 % of good readings for two, `11.34` for
three. Too tight a gate on a filter that has gone wrong will reject everything
it needs to recover, so watch for a long run of rejections.

## Tuning Q and R

`Q` is how much you distrust your model between steps. `R` is how much you
distrust your sensor. Only their ratio matters to the estimate; the absolute
size sets how large `P` settles.

| Knob | Raise it when |
|---|---|
| `Q` | The filter lags real change, or insists on a value the sensor keeps disagreeing with |
| `R` | The estimate is jittery and chases the noise |
| Both, together | Nothing changes. Scale one, not the pair |

By default `Q` is added once per predict, so it is noise per step and a
faster loop is a noisier model. Set `q_per_second` and it is a rate instead,
added as `Q * dt`: tune it once and it holds when the loop period jitters or
changes.

Start with `R` from the sensor's datasheet noise, squared, and `Q` small — small
enough that the filter is smooth, large enough that it still moves. Diagonal is
almost always enough. Both must be symmetric to within `1e-6` and finite, and
`init` refuses them otherwise. `P0` says how wrong `x0` might be: generous if you
are guessing, tight if you know.

## The functions

| Function | What it does |
|---|---|
| `ekf_filter_init` | Sizes, `x0`, `P0`, `Q` and `R`. Copies all four, then clears the callbacks for you to set |
| `ekf_filter_predict` | Runs the model forward by `dt` and grows `P`. `u` may be `NULL` |
| `ekf_filter_update` | Folds in one measurement `z`, Joseph form. `innovation` may be `NULL`, or receives `z - h(x)` |
| `ekf_filter_update_sensor` | The same, for a `struct ekf_sensor` of its own size, model and noise |
| `ekf_filter_reset` | New `x` and `P`, keeping `Q`, `R` and the model |

Every call returns a status. A negative status is an error, and a call that
returns one changes nothing at all. `EKF_FILTER_REJECTED` is positive: not an
error but a verdict, and it changes `nis` and nothing else in the struct. Code
that tests for `!= EKF_FILTER_OK` will treat a rejection as a failure, which is
usually what you want; test `< 0` for errors alone.

| Status | Meaning |
|---|---|
| `EKF_FILTER_REJECTED` | The gate threw the reading out. `x` and `P` untouched |
| `EKF_FILTER_OK` | Succeeded |
| `EKF_FILTER_ERR_NULL` | A required pointer was null |
| `EKF_FILTER_ERR_NOT_FINITE` | A supplied value, or something a callback returned, was not finite |
| `EKF_FILTER_ERR_INVALID_VALUE` | Bad size, missing `f` or `h`, `dt <= 0`, a non-symmetric matrix, or a negative `gate` |
| `EKF_FILTER_ERR_NOT_INITIALIZED` | `init` was never called on this struct |
| `EKF_FILTER_ERR_NUMERIC` | `H P H' + R` could not be inverted, or the result would not fit a `float` |

Predict and update are independent. Run several predicts between measurements
when the sensor is slower than the loop, or several updates in a row for
several sensors, one `struct ekf_sensor` each.

## Sizes and limits

| | |
|---|---|
| States `n` | 1 to 8 |
| Measurements `m` | 1 to 4 |
| Storage | `float` in the struct, 696 bytes on a 64-bit host, all of it yours; 112 per `struct ekf_sensor` |
| Arithmetic | `double` on the stack inside every call, about 4.5 kB of frame for an update |
| Layout | Row-major, packed to `n` and `m`, not to the maximum |

The struct holds no pointer into itself and no hidden state, so it copies with
`=` and lives anywhere you put it. Separate instances are independent;
synchronise a shared one yourself. C++ can use it through the header's
`extern "C"`, which `make check` verifies.

## What it does not do

- **No unscented filter.** The Jacobian is a first-order linearisation. A model
  that bends sharply within one step wants a UKF, which this is not.
- **No square-root form.** `P` is propagated directly, in Joseph form so it
  stays symmetric and positive over a long run — the tests hold it to that over
  10000 cycles. A square-root or UD filter would hold up better still in
  `float`, at more code than this library is trying to be.
- **No adaptive gating or tuning.** The NIS gate is a fixed threshold you
  choose. Nothing re-estimates `Q` or `R`, and nothing re-opens a gate that has
  rejected a filter's way back from a bad state.
- **No timing.** You supply `dt`. With `q_per_second` a jittery loop is no
  longer a mistuned filter, but `dt` still has to be the real one.

## License

MIT
