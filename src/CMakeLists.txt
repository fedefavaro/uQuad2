cmake_minimum_required (VERSION 2.8.10)
project (uQuad2)

# List all dirs with headers
include_directories(common)
include_directories(kernel_msgq)
include_directories(serial_comm)
include_directories(futaba_sbus)

# Add libm, for pow()
link_libraries(m)

# Add warning to compile flags:
set(${PROJECT_NAME}_C_FLAGS "-Wall" CACHE STRING
    "${PROJECT_NAME} C FLAGS")
# Enable the project's flags:
set(CMAKE_C_FLAGS "${${PROJECT_NAME}_C_FLAGS}")

# Generate libs
add_subdirectory(common)
add_subdirectory(kernel_msgq)
add_subdirectory(serial_comm)
add_subdirectory(futaba_sbus)
add_subdirectory(main)
add_subdirectory(sbus_daemon)
