set(sbus_daemon_bin sbus_daemon)
add_executable (${sbus_daemon_bin} sbus_daemon)

# Set required libraries
target_link_libraries(${sbus_daemon_bin} futaba_sbus)
target_link_libraries(${sbus_daemon_bin} serial_comm)
target_link_libraries(${sbus_daemon_bin} custom_baud)
#target_link_libraries(${sbus_daemon_bin} uquad_kernel_msgq)
target_link_libraries(${sbus_daemon_bin} uquad_time)


