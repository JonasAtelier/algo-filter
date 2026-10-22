# algo-filter

<p align="center">
  <img alt="platform: portable C99" src="https://img.shields.io/badge/platform-portable%20C99-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: MIT" src="https://img.shields.io/badge/license-MIT-f2bd68?style=flat-square&labelColor=0d2238">
</p>

State estimation for robots — six small filters in portable C99, each one
header and one `.c` file, no heap. Noisy sensors go in, numbers you can
trust come out.

## Libraries

| Library | What it does | Pick it when |
|---|---|---|
| [kalman](kalman) | Smooths one stream of noisy numbers while still following real change | one sensor, one value — a temperature, an ADC channel |
| [ekf](ekf) | Extended Kalman filter, up to 8 coupled states | the states depend on each other, or the sensor measures them indirectly |
| [madgwick](madgwick) | Gyro + accelerometer (+ magnetometer) into a full orientation | you need all three axes, or yaw |
| [complementary](complementary) | Tilt angle from a gyroscope and an accelerometer | one angle, e.g. a balancing robot, and the least CPU |
| [particle](particle) | Tracks one value with a cloud of weighted guesses | the noise is not Gaussian or the model is non-linear, and you can afford the CPU |
| [rls](rls) | Recursive least squares: learns plant parameters online | you need a number nobody wrote down — `b0` for `adrc`, a motor's friction |

## Use one library

Every library stands on its own: no shared headers, nothing from one library
needed by another. Take the one you need and leave the rest. Three ways in.

**Copy the files.** `kalman/include/kalman_filter.h` and
`kalman/src/kalman_filter.c`
into your project, built with your own sources. That is the whole install.

**CMake.** Every library is its own target, `algo::<folder>`. Add just the
folder you need:

```cmake
add_subdirectory(algo-filter/kalman)
target_link_libraries(app PRIVATE algo::kalman)
```

or, without a copy in your tree:

```cmake
include(FetchContent)
FetchContent_Declare(algo_kalman
    GIT_REPOSITORY https://github.com/JonasAtelier/algo-filter.git
    GIT_TAG        stable
    SOURCE_SUBDIR  kalman)
FetchContent_MakeAvailable(algo_kalman)
target_link_libraries(app PRIVATE algo::kalman)
```

`add_subdirectory(algo-filter)`, or FetchContent without `SOURCE_SUBDIR`, adds
every library's target instead; link the ones you use. Examples and
tests only build when this repo is the top-level project.

**ESP-IDF.** Point `EXTRA_COMPONENT_DIRS` at one library's folder. The
component is named after the folder:

```cmake
# top-level CMakeLists.txt of your IDF project, before project()
list(APPEND EXTRA_COMPONENT_DIRS path/to/algo-filter/kalman)
```

then `REQUIRES kalman` in your component's `idf_component_register()`.

Pointing it at the repo root instead gives you **one** component holding
every library, named after the folder (`algo-filter`): IDF does not look
inside a folder that has its own `CMakeLists.txt`. Use one or the other,
never both — every symbol would be defined twice.

## Build and test

```sh
make check    # every library's own tests, stopping at the first failure
make clean
```

Or with CMake and CTest. CTest runs the C tests only; the C++ linkage tests
run under `make check`.

```sh
cmake -B build && cmake --build build -j2 && ctest --test-dir build
```

## License

MIT. See [LICENSE](LICENSE).
