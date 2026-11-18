# algo-filter

Turn noisy sensor readings into estimates you can use. These six C99 libraries run on a computer or microcontroller.

**Contents** · [Choose a library](#choose-a-library) · [Try the example](#try-the-example) · [Add it to your project](#add-it-to-your-project)

## Choose a library

| Library | What it does | Example and API |
| --- | --- | --- |
| [kalman](kalman/) | Smooth a single stream of readings, such as temperature. | [basic](kalman/examples/basic.c), [custom tuning](kalman/examples/custom_tuning.c) · [API](kalman/include/kalman_filter.h) |
| [complementary](complementary/) | Estimate tilt from a gyroscope and an accelerometer. | [angle](complementary/examples/angle.c) · [API](complementary/include/complementary_filter.h) |
| [madgwick](madgwick/) | Estimate 3D orientation from motion sensors, optionally with a magnetometer. | [tilt](madgwick/examples/tilt.c) · [API](madgwick/include/madgwick_filter.h) |
| [ekf](ekf/) | Track up to eight related quantities using a model of their motion and measurements. | [heading](ekf/examples/heading.c), [pendulum](ekf/examples/pendulum.c) · [API](ekf/include/ekf_filter.h) |
| [particle](particle/) | Track one value using many weighted guesses when a simpler noise model fits poorly. | [track](particle/examples/track.c) · [API](particle/include/particle_filter.h) |
| [rls](rls/) | Learn parameters such as friction from measurements as they arrive. | [motor](rls/examples/motor.c) · [API](rls/include/rls_filter.h) |

## Try the example

From this repository’s root, use Make and a C99 compiler. No robot hardware is needed.

```sh
make -C kalman run
```

The example prints two columns. Compare the raw readings with the smoother filtered estimates. [Example source](kalman/examples/basic.c)

## Add it to your project

For plain C, compile the source files in the library’s `src/` directory and add its `include/` directory to your compiler’s include path. For a CMake application named `app`, place this checkout inside your project and use the following lines.

```cmake
add_subdirectory(algo-filter/kalman)
target_link_libraries(app PRIVATE algo::kalman)
```

For ESP-IDF, add the library directory to `EXTRA_COMPONENT_DIRS` and its name to the consuming component’s `REQUIRES`. Select either the individual libraries or the entire repository to avoid duplicate symbols.

<details>
<summary>Fetch just this library from GitHub with CMake</summary>

```cmake
include(FetchContent)
FetchContent_Declare(algo_kalman
    GIT_REPOSITORY https://github.com/JonasAtelier/algo-filter.git
    GIT_TAG        stable
    SOURCE_SUBDIR  kalman)
FetchContent_MakeAvailable(algo_kalman)
target_link_libraries(app PRIVATE algo::kalman)
```

</details>

Run `make check` from the repository root to test the libraries. [MIT license](LICENSE)

[Browse the workshop catalog · Portable C libraries](https://github.com/JonasAtelier/jonas-atelier-libraries#portable-c-libraries)
