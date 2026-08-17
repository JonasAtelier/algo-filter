# Portable Kalman Library Design

## Goal

Refactor the ESP32-focused repository into a minimal, portable C library for
general applications. The final repository contains only the core source,
README, and portable examples.

## Final Repository Layout

```text
.gitignore
README.md
src/
  kalman_filter.c
  kalman_filter.h
examples/
  basic.c
  custom_tuning.c
```

Arduino packaging, Arduino sketches, tests, validation tools, and the design
documents are removed from the final tree. A small `.gitignore` excludes
generated objects, example binaries, and common editor or operating-system
noise. Existing tests remain available during development and are removed only
after the refactor is verified.

## Portability Contract

The core remains allocation-free and uses caller-owned state. It has no
dependency on Arduino, ESP-IDF, an operating system, threads, or global mutable
state. It uses standard C headers only and supports C callers directly and C++
callers through `extern "C"` declarations.

The source targets C99 or newer and may require linking the platform math
library with `-lm`. Separate filter instances are independent. A caller that
shares one instance between execution contexts is responsible for
synchronization.

## Public API

The public surface contains four operations:

- `kalman_filter_init()` initializes a filter with documented defaults.
- `kalman_filter_init_with_noise()` initializes a filter with custom process
  and measurement noise.
- `kalman_filter_update()` consumes one measurement and writes the filtered
  result.
- `kalman_filter_reset()` preserves tuning while making the next measurement
  the new starting estimate.

`kalman_filter_set_noise()` and `kalman_filter_value()` are removed. This keeps
the common and custom initialization paths without supporting mid-stream
retuning or a redundant getter.

The public filter object remains stack-allocatable. Making it opaque would
require allocation, caller-provided byte storage, or an additional size API,
which would work against the minimal drop-in goal.

## Behavior and Error Handling

The first valid measurement seeds the estimate. Later measurements use the
existing one-dimensional random-walk Kalman calculation and numerically stable
Joseph covariance update.

Initialization validates tuning values. Process noise must be finite and
non-negative; measurement noise must be finite and positive. Updates reject
null pointers, non-finite measurements, invalid filter state, output pointers
that alias filter fields, and non-finite or out-of-range numeric results.

Functions return the existing status enum. On failure, filter state and output
values remain unchanged. The caller must initialize a filter before updating or
resetting it.

## README and Examples

The README is rewritten for generic C users. It introduces what a scalar
Kalman filter does, when it is useful, and how process and measurement noise
affect responsiveness and smoothing. A self-contained Markdown/ASCII flow
diagram and a small raw-versus-filtered table provide the graphical
explanation without adding image assets.

The README also documents direct compilation, linking with `-lm`, the complete
API, error behavior, C++ compatibility, portability limits, and concurrency
expectations.

`examples/basic.c` demonstrates default initialization over a fixed noisy data
set. `examples/custom_tuning.c` runs the same kind of data with explicit tuning.
Both examples use only the library and hosted standard C facilities so they can
be compiled on a normal desktop compiler and adapted to embedded input.

## Verification

Before removing the test tree, update the existing host tests for the reduced
API and run them with strict C11 and C++17 compiler flags. Verification covers
initialization, convergence, reset, invalid arguments, invalid state, numeric
extremes, failure atomicity, and C++ linkage.

Compile both final examples with strict warnings and run them. Inspect the
final file list to confirm that only `.gitignore`, `README.md`, `src/`, and
`examples/` remain. Hardware behavior is not claimed because no
hardware-specific code is part of the final library.

## Out of Scope

- Multidimensional or matrix Kalman filters
- Dynamic allocation or opaque handles
- Platform-specific adapters and build systems
- Runtime retuning
- Thread synchronization
- Changes to the filter equations
