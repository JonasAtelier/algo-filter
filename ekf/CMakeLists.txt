# Makes this library usable directly, instead of needing a wrapper in the
# consuming project: as an ESP-IDF component, or in plain CMake through
# add_subdirectory() or FetchContent as the target algo::ekf.
# Ignored by the Makefile build, which stays the way to build and test this
# library on a host.
if(COMMAND idf_component_register)
    idf_component_register(SRCS "src/ekf_filter.c"
                           INCLUDE_DIRS "include")
    return()
endif()

add_library(algo-ekf src/ekf_filter.c)
add_library(algo::ekf ALIAS algo-ekf)
target_include_directories(algo-ekf PUBLIC include)
set_target_properties(algo-ekf PROPERTIES C_STANDARD 99)
if(NOT MSVC)
    target_link_libraries(algo-ekf PRIVATE m)
endif()
