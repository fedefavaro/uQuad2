
add_library (UAVTalk UAVTalk)

# Set required libraries
target_link_libraries(UAVTalk uquad_time)
target_link_libraries(UAVTalk serial_comm)

