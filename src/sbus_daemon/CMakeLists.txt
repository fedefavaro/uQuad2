set(sbus_daemon_bin sbus_daemon)
add_executable (${sbus_daemon_bin} sbus_daemon)

# Set required libraries
target_link_libraries(${sbus_daemon_bin} futaba_sbus)
target_link_libraries(${sbus_daemon_bin} uquad_time)


