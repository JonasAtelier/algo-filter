# Complementary filter algorithm

Gyroscopes respond quickly, but their integrated angle drifts over time.
Accelerometers provide a stable long-term angle, but vibration and movement
make that angle noisy. A complementary filter combines their strengths.

Conceptually, for every sample this library calculates:

```text
gyro_angle = angle + gyro_rate * dt
angle = gyro_angle + (1 - alpha) * shortest_error(accel_angle, gyro_angle)
```

The gyroscope predicts how the previous angle changed. The accelerometer then
corrects that prediction. The error is wrapped to `[-180, 180)`, so values
such as `179` and `-179` are treated as two degrees apart rather than 358.
Only the correction is wrapped: gyro integration and the returned angle can
continue beyond one revolution.

## Choosing alpha

`alpha` controls how much each sensor contributes:

- `1.0` uses only the gyroscope.
- `0.0` uses only the accelerometer.
- A value such as `0.98` is a common starting point.

The library clamps values outside the range `[0, 1]`. The best value depends
on sensor noise and the sample interval, so tune it for the application. For
a desired correction time constant `tau` and sample interval `dt`, a useful
relationship is:

```text
alpha = tau / (tau + dt)
tau = alpha * dt / (1 - alpha)
```

A larger `alpha` trusts the gyroscope longer; a smaller value corrects toward
the accelerometer faster.

## Units and timing

Use degrees for angles, degrees per second for gyroscope rate, and seconds for
`dt`. Keep the sample interval accurate because gyroscope integration depends
on it. Arduino `micros()` values are microseconds, so divide their unsigned
difference by `1000000.0` to obtain seconds. Unsigned subtraction handles a
single `micros()` counter wrap.

Initialize the filter from the first accelerometer angle when possible. Both
sensors must measure the same axis and use the same sign convention.

Updates with zero, negative, or non-finite `dt`, or with non-finite sensor
values, leave the last finite angle unchanged. Non-finite initial angles use
zero; non-finite reset angles preserve the existing finite angle. Finite
`alpha` values are clamped, while non-finite values use `0.98`. These checks
also repair publicly modified state on each update.

The filter stores only the current angle and `alpha`. It performs no dynamic
allocation and does not read sensors itself.
