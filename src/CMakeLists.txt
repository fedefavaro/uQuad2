cmake_minimum_required (VERSION 2.8.10)
project (uQuad2)

# List all dirs with headers
include_directories(common)
include_directories(kernel_msgq)
include_directories(serial_comm)
include_directories(futaba_sbus)
include_directories(gps)
include_directories(UAVTalk)
include_directories(control_yaw)
include_directories(path_planning)
include_directories(path_following)
include_directories(socket)

# Add libm, for pow()
link_libraries(m)
# Agregar libgps, para comunicacion con gpsd
link_libraries(gps)

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
add_subdirectory(sbus_daemon) #necesario??
add_subdirectory(gps)
add_subdirectory(UAVTalk)
add_subdirectory(control_yaw)
add_subdirectory(path_planning)
add_subdirectory(path_following)
add_subdirectory(socket)
