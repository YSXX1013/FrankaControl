#include<iostream>
#include<thread>
#include<array>
#include<franka/robot.h>
#include<franka/exception.h>
#include<boost/asio/io_context.hpp>
#include<boost/asio/ip/address.hpp>
#include<boost/asio/ip/tcp.hpp>
#include<boost/beast.hpp>

#include"threads.hpp"
#include"parameters.hpp"
#include"websocket_server.hpp"

using namespace std;
using namespace CSIR;

int main(int argc, char** argv){
    // MessageQue<array<double, DOF> > q;
    CommandQueue command_queue;
    RawMessageQueue raw_message_queue;

    condition_variable cv;
    bool if_grasp = false;

    // Setting Boost.Asio and WebSocket Server
    boost::asio::io_context ioc;
    auto const address = boost::asio::ip::make_address("0.0.0.0");
    auto const port = static_cast<unsigned short>(websocket_port);
    std::unique_ptr<franka::Robot> robot_ptr;
    try {
         robot_ptr = std::make_unique<franka::Robot>(robot_ip);
         std::cout << "Connected to robot at " << robot_ip << std::endl;
    } catch (const franka::Exception& e) {
         std::cerr << "Failed to connect to robot: " << e.what() << std::endl;
         return EXIT_FAILURE;
    }
    franka::Robot& robot = *robot_ptr; // 获取引用

    // 创建 WebSocket 服务器
    WebsocketServer ws_server(ioc, boost::asio::ip::tcp::endpoint{address, port}, raw_message_queue);

    // 信号处理
    boost::asio::signal_set signals(ioc, SIGINT, SIGTERM);
    signals.async_wait([&ioc, &ws_server](boost::system::error_code /*ec*/, int /*signo*/) {
        std::cout << "\nReceived stop signal. Stopping..." << std::endl;
        ws_server.stop();
        ioc.stop();
    });

    // 创建并启动线程
    std::vector<std::thread> threads;
    std::cout << "Launching threads..." << std::endl;

    // 机器人控制线程
    threads.emplace_back(thread_robot_control, std::ref(robot), std::ref(command_queue));
    std::cout << " - Robot control thread launched." << std::endl;

    // UDP 接收线程
    threads.emplace_back(thread_upd_recieve, std::ref(command_queue), std::ref(cv), std::ref(if_grasp));
    std::cout << " - UDP receive thread launched." << std::endl;

    // 夹爪控制线程
    threads.emplace_back(thread_gripper_control, std::ref(cv), std::ref(if_grasp));
    std::cout << " - Gripper control thread launched." << std::endl;

    // WebSocket 服务器运行线程
    threads.emplace_back(thread_websocket_server_run, std::ref(ioc));
    std::cout << " - WebSocket server run thread launched." << std::endl;

    // WebSocket 控制命令读取线程
    threads.emplace_back(thread_websocket_command_reader, std::ref(raw_message_queue), std::ref(command_queue), std::ref(cv), std::ref(if_grasp));
    std::cout << " - WebSocket command reader thread launched." << std::endl;

    // WebSocket 机械臂状态反馈线程
    threads.emplace_back(thread_websocket_state_broadcaster, std::ref(robot), std::ref(ws_server));
    std::cout << " - WebSocket state broadcaster thread launched." << std::endl;

    std::cout << "All threads launched. Application running." << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;

    for (auto& t : threads) {
        if (t.joinable()) {
            t.join();
        }
    }

    std::cout << "All threads finished. Exiting." << std::endl;
    return 0;
}
