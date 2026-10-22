# Particle filter algorithm

## What it does

A particle filter estimates a hidden value by tracking many possible values,
called particles. This library tracks one scalar value, such as position,
distance, temperature, or liquid level.

Each cycle has two steps:

1. `particle_filter_predict()` moves every particle by the known movement and
   adds random process noise.
2. `particle_filter_update()` gives more weight to particles near the sensor
   measurement.

The estimate is the weighted average of all particles.

## Storage and lifecycle

The application supplies three `float` arrays of equal length:

- particles hold the possible state values;
- weights hold their probabilities; and
- scratch provides temporary calculation and resampling space.

The arrays must be separate and must not overlap.

The filter performs no dynamic allocation. Three arrays of 100 particles use
about 1.2 KiB when `float` is four bytes.

Initialize the filter once, then call predict and update for every new sample.
Use reset when the state is known to have changed abruptly.

## Noise settings

All values use the same application-defined unit.

- **Initial spread** describes uncertainty in the starting value.
- **Process noise** describes movement that the prediction does not model.
- **Measurement noise** describes typical sensor error.

These values are standard deviations. A setting of `2.0f`, for example, means
that errors of roughly two units are common. Measurement noise must be greater
than zero. Initial spread and process noise may be zero.

Start with 50 to 200 particles. More particles can represent uncertainty more
smoothly, but consume more memory and CPU time. Increase process noise if the
filter reacts too slowly to real state changes. Increase measurement noise if
the estimate follows sensor jitter too closely.

## Weighting and resampling

The update uses a Gaussian measurement model. Logarithms are used while
calculating weights so that very distant measurements do not make every weight
underflow to zero.

When only a few particles retain useful weight, the filter performs systematic
resampling. Likely particles are copied more often and all weights become equal
again. Resampling happens automatically when the effective particle count is
less than half of the configured count.

The internal xorshift pseudo-random generator is small and deterministic. A
fixed seed makes tests and demonstrations repeatable; vary the seed between
devices or runs when repeatability is not wanted. This generator is not
suitable for security or cryptography.

## Limits

This intentionally small library supports a one-dimensional state, additive
movement, and Gaussian noise. Applications needing multi-dimensional motion,
nonlinear model callbacks, particle-specific state structures, or
cryptographic randomness should use a more specialized filter.
