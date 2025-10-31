// main.cpp

#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <array>
#include <mutex>
#include <condition_variable>
#include <string>
#include <atomic> 
#include <memory>
#include <signal.h>
#include <threads.hpp>
#include <parameters.hpp>

#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/robot_state.h>

#include <netinet/in.h>
#include <string.h>

using namespace CSIR;
using namespace std::chrono_literals;

// [!!!] 用于优雅退出的全局原子标志
std::atomic<bool> program_running(true);

void signal_handler(int signum) {
    std::cout << "\nCaught signal " << signum << ". Stopping application..." << std::endl;
    program_running.store(false);
}


int main(int argc, char **argv)
{
    std::cout << "Starting Franka Control Application (UDP Bidirectional)..." << std::endl;

    // --- 注册信号处理器 ---
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // --- 共享资源 ---
    CommandQueue command_queue;
    std::condition_variable gripper_condition;
    bool if_grasp = false;

    // [!!!] UDP 状态反馈所需的共享资源
    std::mutex client_address_mutex;
    struct sockaddr_in shared_client_address;
    memset(&shared_client_address, 0, sizeof(shared_client_address));

    // [!!!] 机器人状态共享资源
    franka::RobotState shared_robot_state;
    std::mutex state_mutex;
    std::atomic<bool> state_updated(false);


    // --- 创建 Franka Robot 对象 ---
    std::unique_ptr<franka::Robot> robot_ptr;
    try {
         robot_ptr = std::make_unique<franka::Robot>(robot_ip);
         std::cout << "Connected to robot at " << robot_ip << std::endl;
    } catch (const franka::Exception& e) {
         std::cerr << "Failed to connect to robot: " << e.what() << std::endl;
         return EXIT_FAILURE;
    }
    franka::Robot& robot = *robot_ptr;

    // --- 创建并启动线程 ---
    std::vector<std::thread> threads;
    std::cout << "Launching threads..." << std::endl;

    try {
        // 1. 机器人控制线程 (传递 robot 和状态共享资源)
        threads.emplace_back(thread_robot_control, 
                             std::ref(robot), 
                             std::ref(command_queue),
                             std::ref(state_mutex),          // [!!!]
                             std::ref(shared_robot_state),   // [!!!]
                             std::ref(state_updated));       // [!!!]
        std::cout << " - Robot control thread launched." << std::endl;

        // 2. UDP 接收线程 (传递命令队列和地址共享资源)
        threads.emplace_back(thread_upd_recieve,
                             std::ref(command_queue),
                             std::ref(gripper_condition),
                             std::ref(if_grasp),
                             std::ref(client_address_mutex),
                             std::ref(shared_client_address));
         std::cout << " - UDP receive thread launched." << std::endl;

        // 3. 夹爪控制线程
        threads.emplace_back(thread_gripper_control,
                             std::ref(gripper_condition),
                             std::ref(if_grasp));
        std::cout << " - Gripper control thread launched." << std::endl;

        // 4. UDP 状态发送线程 (传递地址和状态共享资源)
        threads.emplace_back(thread_udp_send_state,
                             std::ref(client_address_mutex),
                             std::ref(shared_client_address),
                             std::ref(state_mutex),          // [!!!]
                             std::ref(shared_robot_state),   // [!!!]
                             std::ref(state_updated));       // [!!!]
        std::cout << " - UDP state sender thread launched." << std::endl;

    } catch (const std::exception& e) {
         std::cerr << "Failed to launch threads: " << e.what() << std::endl;
         program_running = false;
    }


    std::cout << "All threads launched. Application running." << std::endl;
    
    // [!!!] 主线程等待退出信号
    while (program_running.load()) {
        std::this_thread::sleep_for(100ms);
    }

    std::cout << "Stopping threads..." << std::endl;
    // (注意：[[noreturn]] 线程不会自动停止，这里只是等待它们因异常退出或被外部杀死)
    // (robot.control() 也需要一种停止机制，但 libfranka 没有提供简单的 stop())

    // --- 等待所有线程结束 ---
    for (auto& t : threads) {
        if (t.joinable()) {
            // join() 在这里可能会无限期阻塞，因为线程是无限循环的。
            // 在生产环境中，您需要一种更复杂的机制来优雅地停止所有线程。
            // t.join(); 
        }
    }
    
    // 鉴于线程是无限循环的，我们可能只能在这里强制退出
    // 或者依赖信号处理器来停止整个进程

    std::cout << "Application stopping." << std::endl;
    return 0;
}