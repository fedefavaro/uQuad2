# The extension is already found. Any number of sources could be listed here.
add_library (gps_comm gps_comm)

target_link_libraries(gps_comm serial_comm)
