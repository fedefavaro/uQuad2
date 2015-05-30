cmake_minimum_required (VERSION 2.8.10)
project (uQuad2)

# List all dirs with headers

#link_libraries(??)

# Add warning to compile flags:
set(${PROJECT_NAME}_C_FLAGS "-Wall" CACHE STRING
    "${PROJECT_NAME} C FLAGS")
# Enable the project's flags:
set(CMAKE_C_FLAGS "${${PROJECT_NAME}_C_FLAGS}")

set(main_bin test_socket)
add_executable (${main_bin} main)

# Set required libraries
#target_link_libraries(${main_bin} ??)
