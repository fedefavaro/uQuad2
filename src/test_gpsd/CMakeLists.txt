cmake_minimum_required (VERSION 2.8.10)
project (testGPS)

# Add libm, for pow()
link_libraries(m)
link_libraries(gps)

# Add warning to compile flags:
set(${PROJECT_NAME}_C_FLAGS "-Wall" CACHE STRING
    "${PROJECT_NAME} C FLAGS")
# Enable the project's flags:
set(CMAKE_C_FLAGS "${${PROJECT_NAME}_C_FLAGS}")

# Generate libs
add_subdirectory(main)

