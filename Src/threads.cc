#include "threads.hpp"
#include "udp.hpp"
#include <franka/robot.h>
#include <franka/gripper.h>
#include "robot.hpp"
#include "parameters.hpp"
#include "websocket_server.hpp"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <chrono>
#include <thread>
#include <array>
#include <mutex>

// #include <json.hpp>
#include <boost/json.hpp>

// using namespace std;
// using namespace nlohmann;

void CSIR::thread_robot_control(franka::Robot &robot, CommandQueue &message_queue)
{
    CSIR::Robot::initialize(robot);
    CSIR::Robot::robot_control(robot, message_queue);
}

int CSIR::thread_gripper_control(std::condition_variable &condition, bool &if_grasp)
{

    std::mutex m;
    std::unique_lock<std::mutex> lk(m);

    try
    {
        // initialize
        double grasping_width = 0.02;
        franka::Gripper gripper(robot_ip);

        while (true)
        {
            gripper.homing();
            condition.wait(lk, [&]
                           { return if_grasp; });

            // Check for the maximum grasping width.
            franka::GripperState gripper_state = gripper.readOnce();
            std::cout << gripper_state.max_width << std::endl;
            if (gripper_state.max_width < grasping_width)
            {
                std::cout << "Object is too large for the current fingers on the gripper." << std::endl;
                continue;
            }
            // Grasp the object.
            if (!gripper.grasp(grasping_width, 0.1, 100, 0.05, 0.06))
            {
                std::cout << "Failed to grasp object." << std::endl;
                continue;
            }

            // wait until if the object is still grasped
            while (true)
            {
                gripper_state = gripper.readOnce();
                if (!gripper_state.is_grasped)
                {
                    std::cout << "Object lost" << std::endl;
                    break;
                }

                if (!if_grasp)
                {
                    break;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }

            gripper.stop();
        }
    }
    catch (franka::Exception const &e)
    {
        std::cout << e.what() << std::endl;
        return -1;
    }
}

[[noreturn]] void CSIR::thread_upd_recieve(CommandQueue &message_queue, std::condition_variable &condition, bool &if_grasp)
{
    int len;

    int sock_fd = CSIR::UDP::create_and_bind(udp_port, len);

    ssize_t recv_num;
    char recv_buf[200];
    struct sockaddr_in addr_client;
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100;
    setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

    std::array<double, DOF> JointVal = {0};

    while (true)
    {
        // start server
        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
        if (recv_num > 0)
        {
            recv_buf[recv_num] = '\0';
            std::string data = recv_buf;

            // auto j = json::parse(data);
            // auto joints = j["joints"];
            // double _if_grasp = j["gripper"];

            boost::system::error_code ec;
            boost::json::value j = boost::json::parse(data, ec);
            if (ec)
            {
                std::cerr << "JSON Parse Error: " << ec.message() << " for data: " << data << std::endl;
            }
            else
            {
                try
                {
                    boost::json::array joints = j.at("joints").as_array();
                    bool _if_grasp = j.at("gripper").as_bool();
                    if (joints.size() == DOF)
                    {
                        for (size_t i = 0; i < DOF; ++i)
                        {
                            // Boost.JSON 需要更明确的类型检查和转换 / Boost.JSON requires more explicit type checking and conversion.
                            if (joints[i].is_double())
                            {
                                JointVal[i] = joints[i].as_double();
                            }
                            else if (joints[i].is_int64())
                            {
                                JointVal[i] = static_cast<double>(joints[i].as_int64());
                            }
                            else if (joints[i].is_uint64())
                            {
                                JointVal[i] = static_cast<double>(joints[i].as_uint64());
                            }
                            else
                            {
                                throw std::runtime_error("Invalid joint value type at index " + std::to_string(i));
                            }
                        }
                    }
                    else
                    {
                        std::cerr << "Incorrect number of joints received: " << joints.size() << ", expected " << DOF << std::endl;
                    }
                    // send information
                    // std::cout << JointVal[0]<<' ' <<JointVal[1]<<' '<<JointVal[2]<<' '<<JointVal[3]<<' '<<JointVal[4]<<' '<<JointVal[5]<<' '<<JointVal[6]<< std::endl;
                    message_queue.put(JointVal);

                    // send grasp
                    if_grasp = bool(_if_grasp);
                    condition.notify_all();
                }
                catch (const std::exception &e)
                {
                    std::cerr << "Error accessing JSON data: " << e.what() << '\n';
                }
            }

            // double joint0 = joints[0];
            // double joint1 = joints[1];
            // double joint2 = joints[2];
            // double joint3 = joints[3];
            // double joint4 = joints[4];
            // double joint5 = joints[5];
            // double joint6 = joints[6];
            // JointVal[0] = joint0;
            // JointVal[1] = joint1;
            // JointVal[2] = joint2;
            // JointVal[3] = joint3;
            // JointVal[4] = joint4;
            // JointVal[5] = joint5;
            // JointVal[6] = joint6;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

[[noreturn]] void CSIR::thread_websocket_server_run(boost::asio::io_context &ioc)
{
    try
    {
        std::cout << "Starting WebSocket io_context..." << std::endl;
        ioc.run();                                                 // 运行 io_context，处理所有异步操作
        std::cout << "WebSocket io_context stopped." << std::endl; // 如果 run() 返回，则打印
    }
    catch (const std::exception &e)
    {
        std::cerr << "WebSocket server thread exception: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unknown exception in WebSocket server thread." << std::endl;
    }
    std::cerr << "WebSocket server thread finished unexpectedly." << std::endl;
    std::exit(EXIT_FAILURE); // 发生错误时退出程序
}

[[noreturn]] void CSIR::thread_websocket_state_broadcaster(
    franka::Robot &robot,   // 接收 robot 引用
    WebsocketServer &server // 接收 server 引用
)
{
    // 配置反馈广播频率
    // 100Hz -> 10ms 间隔
    const auto broadcast_interval = std::chrono::milliseconds(10); // 可调整
    // 取决于网络和客户端处理能力，测试后如果可行的话尝试使用更高的频率
    // const auto broadcast_interval = std::chrono::milliseconds(1); // 1000Hz

    std::cout << "WebSocket state broadcaster thread started. Interval: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(broadcast_interval).count() << "ms" << std::endl;

    auto next_broadcast_time = std::chrono::steady_clock::now() + broadcast_interval;

    while (true)
    {
        // 控制循环时间
        std::this_thread::sleep_until(next_broadcast_time);
        next_broadcast_time += broadcast_interval;
        // 如果处理时间超过间隔，立即进行下一次

        try
        {
            // 使用 readOnce() 获取最新状态
            franka::RobotState current_state = robot.readOnce();

            // 序列化
            std::string state_json_str;
            // 使用Boost.Asio
            try
            {
                boost::json::object state_obj;
                boost::json::array joints_array(current_state.q.begin(), current_state.q.end());
                state_obj["current_joints"] = joints_array;

                boost::json::array O_T_EE_array(current_state.O_T_EE.begin(), current_state.O_T_EE.end());
                state_obj["O_T_EE"] = O_T_EE_array;

                // 时间戳
                // auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                //     std::chrono::system_clock::now().time_since_epoch()
                // ).count();
                // state_obj["timestamp_ms"] = timestamp;

                state_json_str = boost::json::serialize(state_obj);
                // 序列化结束

                // 广播
                server.broadcast(state_json_str);
            }
            catch (const std::exception &e_ser)
            {
                std::cerr << "State Broadcaster: Error during JSON serialization: " << e_ser.what() << std::endl;
                // 序列化错误不应停止广播循环，但需要记录
            }
        }
        catch (const franka::Exception &e_read)
        {
            // readOnce() 可能会抛出异常
            std::cerr << "State Broadcaster: Franka exception during readOnce(): " << e_read.what() << std::endl;
            // 短暂休眠以避免在错误状态下持续高频尝试
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            // 重置下次广播时间，避免累积延迟
            next_broadcast_time = std::chrono::steady_clock::now() + broadcast_interval;
        }
        catch (const std::exception &e)
        {
            std::cerr << "State Broadcaster: General error: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            next_broadcast_time = std::chrono::steady_clock::now() + broadcast_interval;
        }
        catch (...)
        {
            std::cerr << "State Broadcaster: Unknown error." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            next_broadcast_time = std::chrono::steady_clock::now() + broadcast_interval;
        }
    } // end while(true)
}
[[noreturn]] void CSIR::thread_websocket_command_reader(
    RawMessageQueue &raw_queue,
    CommandQueue &command_queue,
    std::condition_variable &condition,
    bool &if_grasp)
{
    std::cout << "WebSocket command reader thread started." << std::endl;
    std::array<double, DOF> JointVal = {0}; // 在循环外定义以复用

    while (true)
    {
        std::string raw_message;
        if (raw_queue.get(raw_message))
        { // 尝试从原始队列获取消息
            // std::cout << "Processing raw message: " << raw_message << std::endl; // Debug
            boost::system::error_code ec;
            boost::json::value jv = boost::json::parse(raw_message, ec);

            if (ec)
            {
                std::cerr << "WS Command Reader JSON Parse Error: " << ec.message() << " for data: " << raw_message << std::endl;
            }
            else
            {
                try
                {
                    boost::json::array joints_jv = jv.at("joints").as_array();
                    bool gripper_jv = jv.at("gripper").as_bool();
                    if (joints_jv.size() == DOF)
                    {
                        for (size_t i = 0; i < DOF; ++i)
                        {
                            if (joints_jv[i].is_double())
                            {
                                JointVal[i] = joints_jv[i].as_double();
                            }
                            else if (joints_jv[i].is_int64())
                            {
                                JointVal[i] = static_cast<double>(joints_jv[i].as_int64());
                            }
                            else if (joints_jv[i].is_uint64())
                            {
                                JointVal[i] = static_cast<double>(joints_jv[i].as_uint64());
                            }
                            else
                            {
                                throw std::runtime_error("Invalid joint value type at index " + std::to_string(i));
                            }
                        }
                        // std::cout << "WS Command Parsed: Joints received, putting into queue." << std::endl; // Debug
                        command_queue.put(JointVal); // 放入主命令队列

                        bool old_grasp_state = if_grasp;
                        if_grasp = gripper_jv;
                        // std::cout << "WS Command Parsed: Gripper command: " << if_grasp << std::endl; // Debug
                        if (if_grasp != old_grasp_state)
                        { // 仅当状态改变时通知
                            // std::cout << "WS Command Reader: Notifying gripper condition." << std::endl; // Debug
                            condition.notify_all();
                        }
                    }
                    else
                    {
                        std::cerr << "WS Command Reader: Incorrect number of joints received: " << joints_jv.size() << ", expected " << DOF << std::endl;
                    }
                }
                catch (const std::exception &e)
                {
                    std::cerr << "WS Command Reader Error accessing JSON data: " << e.what() << " Data: " << raw_message << std::endl;
                }
            }
        }
        else
        {
            // 队列为空，短暂休眠避免忙等
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 稍微等待一下
        }
    }
}