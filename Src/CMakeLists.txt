set(CSIR_LIBRARY 
    robot
    threads 
    udp
    examples_common
    websocket_server
)

foreach(library ${CSIR_LIBRARY})
    add_library(${library} ${library}.cc)
endforeach()

target_link_libraries(threads Franka::Franka robot udp websocket_server)
target_link_libraries(robot Franka::Franka examples_common)
target_link_libraries(examples_common Eigen3::Eigen Franka::Franka)