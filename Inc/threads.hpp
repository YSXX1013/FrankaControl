#ifndef _THREADS_HPP_
#define _THREADS_HPP_

#include <typeinfo>
#include <queue>
#include <array>
#include <mutex>
#include <boost/asio/io_context.hpp>
#include <franka/robot.h>

#include "parameters.hpp"
#include <condition_variable>
namespace CSIR
{

    /**
     * @brief simple implementation of a thread-safe message que
     *
     */

    template <class T>
    class MessageQue
    {
    private:
        std::queue<T> _queue;
        std::mutex _q_mutex;

    public:
        MessageQue() : _queue(),
                       _q_mutex()
        {
        }

        /**
         * @brief put a content in que
         *
         * @param array
         */
        void put(T &array)
        {
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
        bool get(T &ret)
        {
            std::lock_guard<std::mutex> guard(_q_mutex);
            if (_queue.empty())
                return false;
            ret = _queue.front();
            _queue.pop();
            return true;
        }

        int size()
        {
            return _queue.size();
        }
    };

    using RawMessageQueue = MessageQue<std::string>; // 复用 MessageQue 实现
    using CommandQueue = MessageQue<std::array<double, DOF>>;
    class WebsocketServer;

    void thread_robot_control(franka::Robot& robot, CommandQueue &message_queue);
    [[noreturn]] void thread_upd_recieve(CommandQueue &message_queue, std::condition_variable &condition, bool &if_grasp);
    int thread_gripper_control(std::condition_variable &condition, bool &if_grasp);
    // Websocket
    [[noreturn]] void thread_websocket_server_run(boost::asio::io_context &ioc);
    [[noreturn]] void thread_websocket_state_broadcaster(franka::Robot &robot, WebsocketServer &server);
    [[noreturn]] void thread_websocket_command_reader(RawMessageQueue& raw_queue,CommandQueue& command_queue,std::condition_variable& condition,bool& if_grasp);
};

#endif