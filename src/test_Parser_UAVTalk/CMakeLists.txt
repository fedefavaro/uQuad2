cmake_minimum_required (VERSION 2.8.10)
project (uQuad2)

# List all dirs with headers

# Add libm
link_libraries(m)

# Add warning to compile flags:
set(${PROJECT_NAME}_C_FLAGS "-Wall" CACHE STRING
    "${PROJECT_NAME} C FLAGS")
# Enable the project's flags:
set(CMAKE_C_FLAGS "${${PROJECT_NAME}_C_FLAGS}")

set(main_bin main)
add_executable (${main_bin} main)

