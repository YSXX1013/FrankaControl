#include "websocket_server.hpp"
#include <boost/json.hpp>
#include <iostream>
#include <vector>
#include <memory> // For std::make_shared

namespace CSIR
{
    // WebsocketSession
    WebsocketSession::WebsocketSession(boost::asio::ip::tcp::socket &&socket, WebsocketServer &server, RawMessageQueue &raw_q)
        : ws_(std::move(socket)),
          server_(server),
          raw_message_queue_(raw_q),
          write_strand_(ws_.get_executor()) // 初始化 strand
    {
    }

    WebsocketSession::~WebsocketSession()
    {
        // std::cout << "WebsocketSession destroyed" << std::endl; // Debug
    }

    void WebsocketSession::run()
    {
        // 将套接字升级到 WebSocket stream
        // 在 strand 上执行 accept 以确保线程安全
        boost::asio::dispatch(ws_.get_executor(),
                              boost::beast::bind_front_handler(
                                  &WebsocketSession::do_accept,
                                  shared_from_this()));
    }

    void WebsocketSession::do_accept()
    {
        // 设置建议的 WebSocket 选项
        ws_.set_option(boost::beast::websocket::stream_base::timeout::suggested(boost::beast::role_type::server));
        ws_.set_option(boost::beast::websocket::stream_base::decorator(
            [](boost::beast::websocket::response_type &res)
            {
                res.set(boost::beast::http::field::server, std::string(BOOST_BEAST_VERSION_STRING) + " websocket-server-async");
            }));

        ws_.async_accept(boost::beast::bind_front_handler(&WebsocketSession::on_accept, shared_from_this()));
    }

    void WebsocketSession::on_accept(boost::beast::error_code ec)
    {
        if (ec)
            return fail(ec, "accept");
        do_read();
    }

    void WebsocketSession::do_read()
    {
        buffer_.consume(buffer_.size());
        ws_.async_read(buffer_, boost::beast::bind_front_handler(&WebsocketSession::on_read, shared_from_this()));
    }

    void WebsocketSession::on_read(boost::beast::error_code ec, std::size_t bytes_transferred)
    {
        boost::ignore_unused(bytes_transferred);

        // 检查是否是正常的 WebSocket 关闭操作
        if (ec == boost::beast::websocket::error::closed)
        {
            std::cout << "WebSocket connection closed normally by peer." << std::endl;
            server_.remove_session(shared_from_this()); // 从服务器移除会话
            return;                                     // 正常关闭，不需要报告错误
        }

        if (ec)
            return fail(ec, "read");

        // 将接收到的消息放入原始消息队列
        std::string received_message = boost::beast::buffers_to_string(buffer_.data());
        // std::cout << "Raw message received: " << received_message << std::endl; // Debug
        raw_message_queue_.put(received_message); // put 需要 string&，但这里是临时string，需要修改MessageQue或创建string

        // 继续读取下一条消息
        do_read();
    }

    // 异步发送消息 - 将消息放入队列并通过 strand 调度写入
    void WebsocketSession::send(std::shared_ptr<const std::string> ss)
    {
        boost::asio::post(write_strand_, [self = shared_from_this(), ss]()
                          {
        // 检查队列是否为空，如果为空则启动写入操作
        bool write_in_progress = !self->write_queue_.empty();
        self->write_queue_.push(ss);
        if (!write_in_progress) {
            self->do_write();
        } });
    }

    void WebsocketSession::do_write()
    {
        // 确保队列中有消息
        if (write_queue_.empty())
            return;

        // 从队列前端获取消息
        auto msg_ptr = write_queue_.front();

        // 异步写入
        // WTF?
        // https://github.com/boostorg/beast/issues/2775
        // Apply the workaround here: wrap write_strand_
        ws_.async_write(boost::asio::buffer(*msg_ptr),
                        boost::asio::bind_executor(
                            boost::asio::any_io_executor{write_strand_}, // <--- Workaround applied here
                            boost::beast::bind_front_handler(
                                &WebsocketSession::on_write,
                                shared_from_this())));
    }

    void WebsocketSession::on_write(boost::beast::error_code ec, std::size_t bytes_transferred)
    {
        boost::ignore_unused(bytes_transferred);

        if (ec)
            return fail(ec, "write");

        // 写入成功，从队列中移除已发送的消息
        write_queue_.pop();

        // 如果队列中还有消息，继续写入
        if (!write_queue_.empty())
        {
            do_write();
        }
    }

    void WebsocketSession::fail(boost::beast::error_code ec, char const *what)
    {
        // 不向对等方发送关闭帧，因为收到了错误
        if (ec != boost::asio::error::operation_aborted && ec != boost::beast::websocket::error::closed)
        {
            std::cerr << "Session Error (" << what << "): " << ec.message() << std::endl;
        }
        // 通知 Server 移除此会话
        server_.remove_session(shared_from_this());
    }

    // WebsocketServer
    WebsocketServer::WebsocketServer(boost::asio::io_context &ioc, boost::asio::ip::tcp::endpoint endpoint, RawMessageQueue &raw_q)
        : ioc_(ioc),
          acceptor_(ioc),
          raw_message_queue_(raw_q)
    {
        boost::beast::error_code ec;

        // 打开 acceptor
        acceptor_.open(endpoint.protocol(), ec);
        if (ec)
        {
            fail(ec, "open");
            return;
        }

        // 允许地址重用
        acceptor_.set_option(boost::asio::socket_base::reuse_address(true), ec);
        if (ec)
        {
            fail(ec, "set_option");
            return;
        }

        // 绑定到服务器地址
        acceptor_.bind(endpoint, ec);
        if (ec)
        {
            fail(ec, "bind");
            return;
        }

        // 开始监听连接
        acceptor_.listen(boost::asio::socket_base::max_listen_connections, ec);
        if (ec)
        {
            fail(ec, "listen");
            return;
        }
        std::cout << "WebSocket server listening on " << endpoint << std::endl;
    }

    void WebsocketServer::run()
    {
        do_accept();
    }

    void WebsocketServer::stop()
    {
        boost::beast::error_code ec;
        acceptor_.close(ec); // 停止接受新连接
        if (ec)
        {
            std::cerr << "Error closing acceptor: " << ec.message() << std::endl;
        }

        // 关闭所有现有连接
        std::vector<std::shared_ptr<WebsocketSession>> sessions_to_close;
        {
            std::lock_guard<std::mutex> lock(sessions_mutex_);
            sessions_to_close.assign(sessions_.begin(), sessions_.end());
            sessions_.clear(); // 清空会话列表
        }
        // 在锁外关闭连接
        for (const auto &session : sessions_to_close)
        {
            // TODO:实现正经的关闭连接
        }
        std::cout << "WebSocket server stopped." << std::endl;
    }

    void WebsocketServer::do_accept()
    {
        acceptor_.async_accept(
            boost::asio::make_strand(ioc_), // 在 strand 上处理 accept
            boost::beast::bind_front_handler(
                &WebsocketServer::on_accept,
                this)); // this 指针
    }

    void WebsocketServer::on_accept(boost::beast::error_code ec, boost::asio::ip::tcp::socket socket)
    {
        if (ec)
        {
            fail(ec, "accept");
            // if (ec != net::error::operation_aborted)
            //     do_accept();
        }
        else
        {
            std::cout << "WebSocket connection accepted from " << socket.remote_endpoint() << std::endl;
            // 创建新的会话并启动它
            auto session = std::make_shared<WebsocketSession>(std::move(socket), *this, raw_message_queue_);
            add_session(session);
            session->run();
        }

        // 持续接受下一个连接，除非 acceptor 已关闭
        if (acceptor_.is_open())
        {
            do_accept();
        }
    }

    void WebsocketServer::add_session(std::shared_ptr<WebsocketSession> session)
    {
        std::lock_guard<std::mutex> lock(sessions_mutex_);
        sessions_.insert(session);
        std::cout << "WebSocket session added. Active sessions: " << sessions_.size() << std::endl;
    }

    void WebsocketServer::remove_session(std::shared_ptr<WebsocketSession> session)
    {
        std::lock_guard<std::mutex> lock(sessions_mutex_);
        sessions_.erase(session);
        std::cout << "WebSocket session removed. Active sessions: " << sessions_.size() << std::endl;
    }

    // 广播消息给所有会话
    void WebsocketServer::broadcast(const std::string &message)
    {
        auto const ss = std::make_shared<const std::string>(message);

        std::vector<std::shared_ptr<WebsocketSession>> sessions_copy;
        {
            std::lock_guard<std::mutex> lock(sessions_mutex_);
            sessions_copy.reserve(sessions_.size());
            for (const auto &weak_session_ptr : sessions_)
            {
                sessions_copy.push_back(weak_session_ptr);
                // 如果使用 weak_ptr，需要 lock()
                // if (auto session_ptr = weak_session_ptr.lock()) {
                //    sessions_copy.push_back(session_ptr);
                // }
            }
        }

        for (const auto &session : sessions_copy)
        {
            boost::asio::post(session->write_strand_, [session, ss]()
                              { session->send(ss); });
        }
    }

    void WebsocketServer::fail(boost::beast::error_code ec, char const *what)
    {
        std::cerr << what << ": " << ec.message() << "\n";
    }

} // namespace CSIR