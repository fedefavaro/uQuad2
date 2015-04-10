set(main_bin test_path_planning)
add_executable (${main_bin} main)

# Set required libraries
target_link_libraries(${main_bin} path_planning)



