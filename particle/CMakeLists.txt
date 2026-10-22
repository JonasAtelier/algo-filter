# Makes this library usable directly, instead of needing a wrapper in the
# consuming project: as an ESP-IDF component, or in plain CMake through
# add_subdirectory() or FetchContent as the target algo::particle.
# Ignored by the Makefile build, which stays the way to build and test this
# library on a host.
if(COMMAND idf_component_register)
    idf_component_register(SRCS "src/particle_filter.c"
                           INCLUDE_DIRS "include")
    return()
endif()

add_library(algo-particle src/particle_filter.c)
add_library(algo::particle ALIAS algo-particle)
target_include_directories(algo-particle PUBLIC include)
set_target_properties(algo-particle PROPERTIES C_STANDARD 99)
if(NOT MSVC)
    target_link_libraries(algo-particle PRIVATE m)
endif()
