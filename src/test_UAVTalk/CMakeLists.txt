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

add_library (serial_comm serial_comm)
add_library (UAVTalk UAVTalk)

set(main_bin test_UAVTalk)
add_executable (${main_bin} main)

# Set required libraries
target_link_libraries(${main_bin} serial_comm)
target_link_libraries(${main_bin} UAVTalk)
