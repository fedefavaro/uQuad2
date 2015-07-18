cmake_minimum_required (VERSION 2.8.10)
project (uQuad2 C CXX)

# List all dirs with headers

# Add libm
link_libraries(m)

# Add warning to compile flags:
set(${PROJECT_NAME}_C_FLAGS "-Wall" CACHE STRING
    "${PROJECT_NAME} C FLAGS")
# Enable the project's flags:
set(CMAKE_C_FLAGS "${${PROJECT_NAME}_C_FLAGS}")

find_package (Threads)
set(main_bin main)
add_executable (${main_bin} main)
target_link_libraries (${main_bin} ${CMAKE_THREAD_LIBS_INIT})


add_executable (shm_client shm_client)
target_link_libraries (shm_client ${CMAKE_THREAD_LIBS_INIT})
