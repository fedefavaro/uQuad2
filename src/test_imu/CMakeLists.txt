cmake_minimum_required (VERSION 2.8.10)
project (test_IMU)

set(main_bin test_imu)
add_executable (${main_bin} main)

link_libraries(m)

add_library(uquad_aux_math uquad_aux_math)

add_library (uquad_time uquad_aux_time)

add_library (imu_comm imu_comm)
target_link_libraries(imu_comm uquad_aux_math)
target_link_libraries(imu_comm uquad_time)

target_link_libraries(${main_bin} imu_comm)
target_link_libraries(${main_bin} uquad_time)
