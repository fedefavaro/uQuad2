set(main_bin main)
add_executable (${main_bin} main)

# Set required libraries
target_link_libraries(${main_bin} uquad_kernel_msgq)
target_link_libraries(${main_bin} uquad_time)


