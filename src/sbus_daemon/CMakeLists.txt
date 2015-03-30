set(sbusd sbus_daemon)
add_executable (${sbusd} sbus_daemon)

# Set required libraries
target_link_libraries(${sbusd} futaba_sbus)
target_link_libraries(${sbusd} serial_comm)
target_link_libraries(${sbusd} custom_baud)
#target_link_libraries(${sbusd} uquad_kernel_msgq)
target_link_libraries(${sbusd} uquad_time)


