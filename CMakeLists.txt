cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_sdk)

find_package(Eigen3)
find_package(Python3 REQUIRED)
find_package (Python3 COMPONENTS Interpreter Development)

include_directories(
    include 
    ${EIGEN3_INCLUDE_DIR}
)

link_directories(lib)

add_compile_options(-std=c++11)

set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)

set(CMAKE_CXX_FLAGS "-O3")

# one pc one process
add_executable(example_position examples/example_position.cpp)
target_link_libraries(example_position ${EXTRA_LIBS})

add_executable(example_velocity examples/example_velocity.cpp)
target_link_libraries(example_velocity ${EXTRA_LIBS})

add_executable(example_torque examples/example_torque.cpp)
target_link_libraries(example_torque ${EXTRA_LIBS})

add_executable(example_walk examples/example_walk.cpp)
target_link_libraries(example_walk ${EXTRA_LIBS})

add_executable(example_dance examples/example_dance.cpp)
target_link_libraries(example_dance ${EXTRA_LIBS})

add_executable(example_wirelessHandle examples/example_wirelessHandle.cpp)
target_link_libraries(example_wirelessHandle ${EXTRA_LIBS})

# one pc multi process
add_executable(lcm_server_high examples/lcm_server_high.cpp)
target_link_libraries(lcm_server_high ${EXTRA_LIBS})

add_executable(lcm_server_low examples/lcm_server_low.cpp)
target_link_libraries(lcm_server_low ${EXTRA_LIBS})

# multi pc
add_executable(udp_send_test examples/multi_pc_udp_send.cpp)
target_link_libraries(udp_send_test ${EXTRA_LIBS})

add_executable(udp_recv_test examples/multi_pc_udp_recv.cpp)
target_link_libraries(udp_recv_test ${EXTRA_LIBS})

# lcm server
add_executable(lcm_server examples/lcm_server.cpp)
target_link_libraries(lcm_server ${EXTRA_LIBS})

# sw examples 
add_executable(sw_example_torque examples/sw_example_torque.cpp)
target_link_libraries(sw_example_torque ${EXTRA_LIBS})

add_executable(sw_example_motor_state examples/sw_example_motor_state.cpp)
target_link_libraries(sw_example_motor_state ${EXTRA_LIBS})

add_executable(sw_example_motor_velocity examples/sw_example_motor_velocity.cpp)
target_include_directories(sw_example_motor_velocity PRIVATE ${Python3_INCLUDE_DIRS})
target_link_libraries(sw_example_motor_velocity ${EXTRA_LIBS} Python3::Python)

add_executable(sw_example_foot_force examples/sw_example_foot_force.cpp)
target_link_libraries(sw_example_foot_force ${EXTRA_LIBS})

add_executable(sw_example_imu examples/sw_example_imu.cpp)
target_link_libraries(sw_example_imu ${EXTRA_LIBS})

# utils examples
add_executable(eigen_cwiseProduct utils_examples/eigen_cwiseProduct.cpp)
target_link_libraries(eigen_cwiseProduct Eigen3::Eigen)

add_executable(eigen_block utils_examples/eigen_block.cpp)
target_link_libraries(eigen_block Eigen3::Eigen)
