# The extension is already found. Any number of sources could be listed here.
add_library (imu_comm imu_comm)

target_link_libraries(imu_comm uquad_aux_math)
