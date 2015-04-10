cmake_minimum_required (VERSION 2.8.10)
project (uQuad2)

# List all dirs with headers
include_directories(path_planning)

# Add libm, for pow()
link_libraries(m)

# Add warning to compile flags:
set(${PROJECT_NAME}_C_FLAGS "-Wall" CACHE STRING
    "${PROJECT_NAME} C FLAGS")
# Enable the project's flags:
set(CMAKE_C_FLAGS "${${PROJECT_NAME}_C_FLAGS}")

# Generate libs
add_subdirectory(path_planning)
add_subdirectory(main)

