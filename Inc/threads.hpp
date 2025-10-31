#ifndef _THREADS_HPP_
#define _THREADS_HPP_

#include<typeinfo>
#include<queue>
#include<array>
#include<mutex>

#include <franka/robot.h> // 添加
#include <franka/robot_state.h> // 添加
#include <netinet/in.h> // 添加, 为了 struct sockaddr_in

#include<parameters.hpp>
#include<condition_variable>
namespace CSIR{

/**
 * @brief simple implementation of a thread-safe message que
 * 
 */

template <class T>
class MessageQue{
    private:
    std::queue<T> _queue;
    std::mutex _q_mutex;

    public:
    MessageQue():
    _queue(),
    _q_mutex()
    {}

    /**
     * @brief put a content in que
     * 
     * @param array 
     */
    void put(T &array){
        std::lock_guard<std::mutex> guard(_q_mutex);
        _queue.push(array);
    }

    /**
     * @brief Get the Array objects 
     * 
     * @param ret the return content
     * @return true get the content successfully
     * @return false if the que is empty
     */
    bool get(T &ret){
        std::lock_guard<std::mutex> guard(_q_mutex);
        if(_queue.empty()) return false;
        ret = _queue.front();
        _queue.pop();
        return true;
    }

    int size(){
        return _queue.size();
    }
};

using CommandQueue = MessageQue<std::array<double, DOF>>;

/**
 * @brief 机器人实时控制线程
 */
void thread_robot_control(
    franka::Robot& robot,
    CommandQueue& command_queue,
    std::mutex& state_mutex,
    franka::RobotState& shared_state,
    std::atomic<bool>& state_updated
);

/**
 * @brief UDP 接收线程 (修改：添加共享地址参数)
 */
[[noreturn]] void thread_upd_recieve(
    CommandQueue& message_queue,
    std::condition_variable& condition,
    bool& if_grasp,
    std::mutex& client_addr_mutex,
    struct sockaddr_in& shared_client_addr
);

/**
 * @brief 夹爪控制线程
 */
int thread_gripper_control(
    std::condition_variable& condition,
    bool& if_grasp
);

/**
 * @brief UDP 状态发送线程
 */
[[noreturn]] void thread_udp_send_state(
    // franka::Robot& robot,
    std::mutex& client_addr_mutex,
    struct sockaddr_in& shared_client_addr,
    std::mutex& state_mutex,
    franka::RobotState& shared_state,
    std::atomic<bool>& state_updated
);

};

#endif