#ifndef _WEBSOCKET_SERVER_HPP_
#define _WEBSOCKET_SERVER_HPP_

#include <threads.hpp>          // 为了 MessageQue 和 DOF
#include <parameters.hpp>       // 为了端口号
#include <franka/robot_state.h> // 为了共享状态

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/dispatch.hpp>
#include <boost/asio/strand.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/signal_set.hpp>

#include <atomic>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <set>
#include <mutex>
#include <queue> // 用于原始消息队列

namespace CSIR
{
    // 前向声明
    class WebsocketSession;
    class WebsocketServer;

    // --- Websocket 会话类 ---
    // 处理单个 WebSocket 连接的读写
    class WebsocketSession : public std::enable_shared_from_this<WebsocketSession>
    {
        boost::beast::websocket::stream<boost::beast::tcp_stream> ws_;
        boost::beast::flat_buffer buffer_;
        WebsocketServer &server_;            // 回调 Server 以便移除自身和访问共享资源
        RawMessageQueue &raw_message_queue_; // 接收到的原始消息放入此队列

        // 用于安全发送的 Strand
        // boost::asio::strand<boost::asio::any_io_executor> write_strand_;
        std::queue<std::shared_ptr<const std::string>> write_queue_; // 发送队列

    public:
        WebsocketSession(boost::asio::ip::tcp::socket &&socket, WebsocketServer &server, RawMessageQueue &raw_q);
        ~WebsocketSession(); // 添加析构函数声明

        // 启动会话（接受握手）
        void run();

        // 异步发送消息
        void send(std::shared_ptr<const std::string> ss);
        boost::asio::strand<boost::asio::any_io_executor> write_strand_;

    private:
        void do_accept();
        void on_accept(boost::beast::error_code ec);
        void do_read();
        void on_read(boost::beast::error_code ec, std::size_t bytes_transferred);
        void do_write();
        void on_write(boost::beast::error_code ec, std::size_t bytes_transferred);
        void fail(boost::beast::error_code ec, char const *what);
    };

    // --- Websocket 服务器类 ---
    // 接受连接并管理会话
    class WebsocketServer
    {
        friend class WebsocketSession; // 允许 Session 访问 Server 成员

        boost::asio::io_context &ioc_;
        boost::asio::ip::tcp::acceptor acceptor_;
        RawMessageQueue &raw_message_queue_; // 传递给 Session

        std::set<std::shared_ptr<WebsocketSession>> sessions_; // 活跃的会话
        std::mutex sessions_mutex_;                            // 保护 sessions_ 集合
        boost::asio::strand<boost::asio::any_io_executor> write_strand_;

    public:
        WebsocketServer(boost::asio::io_context &ioc, boost::asio::ip::tcp::endpoint endpoint, RawMessageQueue &raw_q);

        // 启动服务器（开始接受连接）
        void run();

        // 停止服务器
        void stop();

        // 广播消息给所有连接的客户端
        void broadcast(const std::string &message);

    private:
        void do_accept();
        void on_accept(boost::beast::error_code ec, boost::asio::ip::tcp::socket socket);
        void add_session(std::shared_ptr<WebsocketSession> session);
        void remove_session(std::shared_ptr<WebsocketSession> session);
        void fail(boost::beast::error_code ec, char const *what);
    };
} // namespace CSIR

#endif // _WEBSOCKET_SERVER_HPP_