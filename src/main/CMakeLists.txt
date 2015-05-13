set(main_bin path_follower)
add_executable (${main_bin} main)

# Set required libraries
target_link_libraries(${main_bin} uquad_kernel_msgq)
target_link_libraries(${main_bin} uquad_time)
target_link_libraries(${main_bin} uquad_io)
target_link_libraries(${main_bin} futaba_sbus)
target_link_libraries(${main_bin} gps_comm)
target_link_libraries(${main_bin} UAVTalk)
