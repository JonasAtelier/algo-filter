# algo-filter

Turn noisy sensor readings into estimates you can use. These six C99 libraries run on a computer or microcontroller.

## Try the example

From this repository’s root, use Make and a C99 compiler. No robot hardware is needed.

```sh
make -C kalman run
```

The example prints two columns. Compare the raw readings with the smoother filtered estimates. [Example source](kalman/examples/basic.c)

## Choose a library

| Library | What it does |
| --- | --- |
| [kalman](kalman/) | Smooth a single stream of readings, such as temperature. |
| [complementary](complementary/) | Estimate tilt from a gyroscope and an accelerometer. |
| [madgwick](madgwick/) | Estimate 3D orientation from motion sensors, optionally with a magnetometer. |
| [ekf](ekf/) | Track up to eight related quantities using a model of their motion and measurements. |
| [particle](particle/) | Track one value using many weighted guesses when a simpler noise model fits poorly. |
| [rls](rls/) | Learn parameters such as friction from measurements as they arrive. |

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
