cmake_minimum_required(VERSION 3.30.0)
project(csir_franka_control VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(main main.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)


#find essentiao package
find_package(Eigen3 REQUIRED)
find_package(Franka REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost 1.70.0 REQUIRED COMPONENTS system thread json)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    message(STATUS "Boost found (includes JSON, System, Thread)")
else()
    message(FATAL_ERROR "Boost >= 1.70 (with system, thread, json) not found.")
endif()
include_directories(Inc)

#compile library
add_subdirectory(Src)
add_subdirectory(Test)


#
target_link_libraries(main Threads::Threads threads ${Boost_LIBRARIES})

