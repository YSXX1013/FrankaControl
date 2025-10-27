#include <threads.hpp>
#include <udp.hpp>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <robot.hpp>
#include <parameters.hpp>

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

void CSIR::thread_robot_control(const char *robot_ip_, MessageQue<std::array<double, DOF>> &message_queue)
{
    franka::Robot robot(robot_ip_);
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

[[noreturn]] void CSIR::thread_upd_recieve(MessageQue<std::array<double, DOF>> &message_queue, std::condition_variable &condition, bool &if_grasp)
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