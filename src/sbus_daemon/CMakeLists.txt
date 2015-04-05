set(sbus_daemon sbusd)
add_executable (${sbus_daemon} sbus_daemon)

# Set required libraries
target_link_libraries(${sbus_daemon} futaba_sbus)
target_link_libraries(${sbus_daemon} serial_comm)
target_link_libraries(${sbus_daemon} custom_baud)
target_link_libraries(${sbus_daemon} uquad_kernel_msgq)
target_link_libraries(${sbus_daemon} uquad_time)


