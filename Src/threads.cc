// Src/threads.cc
#include "threads.hpp"
#include "udp.hpp"
#include "robot.hpp"
#include "parameters.hpp"

#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/robot_state.h> // [!!!] 确保包含
#include <franka/exception.h>   // [!!!] 确保包含

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h> // [!!!] 需要
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

#include <chrono>
#include <thread>
#include <array>
#include <mutex>
#include <iostream> // [!!!] 添加

#include <boost/json.hpp>

// 使用 using namespace std::chrono_literals; 来简化时间表示 (如 100ms)
using namespace std::chrono_literals;

namespace CSIR
{

    // --- 1. 修改 thread_robot_control ---
    // (它不再需要处理状态共享，只处理控制)
    void thread_robot_control(
        franka::Robot &robot, // [!!!] 使用 Robot 引用
        CommandQueue &command_queue,
        std::mutex &state_mutex,
        franka::RobotState &shared_state,
        std::atomic<bool> &state_updated)
    {
        try
        {
            Robot::initialize(robot);
            std::cout << "Robot control loop started" << std::endl;

            std::queue<double> desire_angle_queues[DOF];
            PID pid_optimizers[DOF];
            for (int i = 0; i < DOF; i++)
            {
                pid_optimizers[i] = PID(time_control_interval / 1000.0,
                                        max_q_d[i], 0.0 - max_q_d[i],
                                        Kp, Kd, Ki);
            }
            uint64_t time_update_count = 0;
            Robot::arrayDOF previous_setting_angles = INITIAL_ANGLE_STATE;

            robot.control([&](const franka::RobotState &robot_state, franka::Duration period) -> franka::JointVelocities
                          {
            
            // ... (原有的控制逻辑保持不变) ...
            time_update_count += period.toMSec();
            if(time_update_count >= time_update_direvalue){
                Robot::update_desire_velocity(desire_angle_queues, previous_setting_angles, command_queue);
                time_update_count = 0;
            }
            franka::JointVelocities ret = Robot::operate_pid(robot_state,
                                                      desire_angle_queues,
                                                      pid_optimizers,
                                                      previous_setting_angles);

            // [!!!] 新增：更新共享状态
            {
                // 持有锁的时间非常短
                std::lock_guard<std::mutex> lock(state_mutex);
                shared_state = robot_state; // 快速复制状态
            }
            state_updated.store(true, std::memory_order_release); // 通知发送线程
            // [!!!] 状态更新结束

            return ret; });
        }
        catch (const franka::Exception &e)
        {
            std::cerr << "Franka exception in control thread: " << e.what() << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Standard exception in control thread: " << e.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << "Unknown exception in control thread." << std::endl;
        }
        std::cerr << "Robot control loop finished unexpectedly." << std::endl;
    }

    // --- 2. 修改 thread_upd_recieve ---
    // (添加共享地址的更新)
    [[noreturn]] void thread_upd_recieve(
        CommandQueue &message_queue,
        std::condition_variable &condition,
        bool &if_grasp,
        std::mutex &client_addr_mutex,         // [!!!] 新增
        struct sockaddr_in &shared_client_addr // [!!!] 新增
    )
    {
        int len = 0;
        int sock_fd = -1;
        try
        {
            sock_fd = UDP::create_and_bind(udp_port, len);
            if (sock_fd < 0)
            {
                throw std::runtime_error("Failed to create and bind UDP socket");
            }
            std::cout << "UDP command server listening on port " << udp_port << std::endl;

            ssize_t recv_num;
            char recv_buf[512]; // 增大缓冲区
            struct sockaddr_in addr_client;
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 100000; // 100ms 超时
            setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

            std::array<double, DOF> JointVal = {0};

            while (true)
            {
                memset(&addr_client, 0, sizeof(addr_client));
                socklen_t client_len = sizeof(addr_client);

                recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf) - 1, 0, (struct sockaddr *)&addr_client, &client_len);

                if (recv_num > 0)
                {
                    recv_buf[recv_num] = '\0';
                    std::string data = recv_buf;

                    // [!!!] 收到数据包，立即更新客户端地址
                    {
                        std::lock_guard<std::mutex> lock(client_addr_mutex);
                        shared_client_addr = addr_client; // 存储客户端地址
                    }

                    // ... (Boost.JSON 解析逻辑保持不变) ...
                    boost::system::error_code ec;
                    boost::json::value j = boost::json::parse(data, ec);
                    if (ec)
                    {
                        std::cerr << "UDP JSON Parse Error: " << ec.message() << " for data: " << data << std::endl;
                    }
                    else
                    {
                        try
                        {
                            boost::json::array joints = j.at("joints").as_array();
                            bool _if_grasp = j.at("gripper").as_bool(); // 假设 Boost >= 1.80

                            if (joints.size() == DOF)
                            {
                                for (size_t i = 0; i < DOF; ++i)
                                {
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
                                        throw std::runtime_error("Invalid joint value type");
                                    }
                                }
                                message_queue.put(JointVal);

                                bool old_grasp_state = if_grasp;
                                if_grasp = bool(_if_grasp);
                                if (if_grasp != old_grasp_state)
                                {
                                    condition.notify_all();
                                }
                            }
                            else
                            {
                                std::cerr << "UDP: Incorrect number of joints received." << std::endl;
                            }
                        }
                        catch (const std::exception &e)
                        {
                            std::cerr << "UDP Error accessing JSON data: " << e.what() << '\n';
                        }
                    }
                }
                else if (recv_num < 0) // 错误或超时
                {
                    if (errno != EAGAIN && errno != EWOULDBLOCK)
                    {
                        perror("UDP recvfrom error");
                        std::this_thread::sleep_for(10ms); // 发生错误时短暂休眠
                    }
                    // 超时是正常的，继续循环
                }
            } // end while(true)
        }
        catch (const std::exception &e)
        {
            std::cerr << "UDP receive thread exception: " << e.what() << std::endl;
            if (sock_fd >= 0)
                close(sock_fd);
            std::exit(EXIT_FAILURE);
        }
        catch (...)
        {
            std::cerr << "Unknown exception in UDP receive thread." << std::endl;
            if (sock_fd >= 0)
                close(sock_fd);
            std::exit(EXIT_FAILURE);
        }
    }

    // --- 3. thread_gripper_control 保持不变 ---
    // (从您上次上传的文件中复制)
    int thread_gripper_control(std::condition_variable &condition, bool &if_grasp)
    {
        std::mutex m;
        std::unique_lock<std::mutex> lk(m);
        franka::Gripper *gripper_ptr = nullptr;

        try
        {
            double grasping_width = 0.02;
            franka::Gripper gripper(robot_ip);
            gripper_ptr = &gripper;

            while (true)
            {
                gripper.homing();
                std::cout << "Gripper: Homing and waiting..." << std::endl;
                condition.wait(lk, [&]
                               { return if_grasp; });
                std::cout << "Gripper: Grasp command received." << std::endl;

                franka::GripperState gripper_state = gripper.readOnce();
                // std::cout << gripper_state.max_width << std::endl;
                if (gripper_state.max_width < grasping_width)
                {
                    std::cout << "Object is too large for the current fingers on the gripper." << std::endl;
                    if_grasp = false; // 重置
                    continue;
                }

                if (!gripper.grasp(grasping_width, 0.1, 100, 0.05, 0.06))
                {
                    std::cout << "Failed to grasp object." << std::endl;
                    if_grasp = false; // 重置
                    gripper.stop();
                    gripper.move(gripper_state.max_width, 0.1); // 尝试打开
                    continue;
                }
                std::cout << "Gripper: Grasp successful. Holding..." << std::endl;

                // 保持抓取，直到 if_grasp 变为 false 或物体丢失
                while (true)
                {
                    gripper_state = gripper.readOnce();
                    if (!gripper_state.is_grasped)
                    {
                        std::cout << "Object lost" << std::endl;
                        if_grasp = false; // 确保状态同步
                        break;
                    }

                    // 使用 wait_for 检查 if_grasp 状态
                    if (condition.wait_for(lk, 300ms, [&]
                                           { return !if_grasp; }))
                    {
                        // if_grasp 变为 false，退出
                        std::cout << "Gripper: Release command received." << std::endl;
                        break;
                    }
                    // 超时，继续循环检查 is_grasped
                }

                std::cout << "Gripper: Releasing/Stopping..." << std::endl;
                gripper.stop();
                // 释放后自动打开
                gripper.move(gripper_state.max_width, 0.1);
                // if_grasp 已经是 false
            }
        }
        catch (franka::Exception const &e)
        {
            std::cout << "Gripper thread exception: " << e.what() << std::endl;
            if (gripper_ptr)
                gripper_ptr->stop();
            return -1;
        }
        return 0; // 理论上不会到这里
    }

    // --- 4. [!!!] 新增 thread_udp_send_state ---
    [[noreturn]] void thread_udp_send_state(
        std::mutex &client_addr_mutex,
        struct sockaddr_in &shared_client_addr,
        std::mutex &state_mutex,          // [!!!]
        franka::RobotState &shared_state, // [!!!]
        std::atomic<bool> &state_updated  // [!!!]
    )
    {
        const auto broadcast_interval = 10ms; // 100Hz
        std::cout << "UDP state sender thread started. Interval: 10ms" << std::endl;

        int send_sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (send_sock_fd < 0)
        {
            perror("UDP state sender: socket creation failed");
            std::exit(EXIT_FAILURE);
        }

        struct sockaddr_in client_addr_to_send;
        bool client_known = false;

        // 本地状态副本，用于序列化
        franka::RobotState local_state_copy;

        auto next_broadcast_time = std::chrono::steady_clock::now() + broadcast_interval;

        while (true)
        {
            std::this_thread::sleep_until(next_broadcast_time);
            next_broadcast_time += broadcast_interval;

            // 检查是否有新状态
            if (!state_updated.load(std::memory_order_acquire))
            {
                continue; // 没有新状态，跳过此次广播
            }

            try
            {
                // [!!!] 1. 获取最新状态（从共享变量）
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    local_state_copy = shared_state; // 快速复制
                }
                state_updated.store(false, std::memory_order_relaxed); // 重置标志

                // [!!!] 2. 检查客户端地址
                if (!client_known)
                {
                    std::lock_guard<std::mutex> lock(client_addr_mutex);
                    if (shared_client_addr.sin_family == AF_INET)
                    {
                        client_addr_to_send = shared_client_addr;
                        client_addr_to_send.sin_port = htons(udp_state_port);
                        client_known = true;
                        std::cout << "UDP state sender: Client address acquired: "
                                  << inet_ntoa(client_addr_to_send.sin_addr) << ":"
                                  << ntohs(client_addr_to_send.sin_port) << std::endl;
                    }
                    else
                    {
                        continue; // 客户端未知，跳过
                    }
                }

                // [!!!] 3. 序列化（使用 local_state_copy）
                std::string state_json_str;
                try
                {
                    boost::json::object state_obj;
                    boost::json::array joints_array(local_state_copy.q.begin(), local_state_copy.q.end());
                    state_obj["current_joints"] = joints_array;

                    boost::json::array O_T_EE_array(local_state_copy.O_T_EE.begin(), local_state_copy.O_T_EE.end());
                    state_obj["O_T_EE"] = O_T_EE_array;

                    state_json_str = boost::json::serialize(state_obj);
                }
                catch (const std::exception &e_ser)
                {
                    std::cerr << "State Sender: Error during JSON serialization: " << e_ser.what() << std::endl;
                    continue;
                }

                // [!!!] 4. 发送 UDP 包
                ssize_t sent_bytes = sendto(send_sock_fd,
                                            state_json_str.c_str(),
                                            state_json_str.length(),
                                            0,
                                            (struct sockaddr *)&client_addr_to_send,
                                            sizeof(client_addr_to_send));

                if (sent_bytes < 0)
                {
                    perror("UDP state sender: sendto failed");
                    std::lock_guard<std::mutex> lock(client_addr_mutex);
                    shared_client_addr.sin_family = 0;
                    client_known = false;
                    std::cout << "UDP state sender: Client address reset." << std::endl;
                }
            }
            catch (const std::exception &e)
            { // 捕获常规异常
                std::cerr << "State Sender: General error: " << e.what() << std::endl;
                std::this_thread::sleep_for(500ms);
                next_broadcast_time = std::chrono::steady_clock::now() + broadcast_interval;
            }
            catch (...)
            {
                std::cerr << "State Sender: Unknown error." << std::endl;
                std::this_thread::sleep_for(1s);
                next_broadcast_time = std::chrono::steady_clock::now() + broadcast_interval;
            }
        } // end while(true)
    }

} // namespace CSIR