
#include <iostream>
#include <boost/asio.hpp>
#include <functional>

using namespace boost::asio;
using namespace std::placeholders;

const std::size_t STDIN_BUFFER_SIZE = 100;

/*
void handle_arduino_response(
        const boost::system::error_code& ec,
        std::size_t bytes_transferred,
        posix::stream_descriptor outstream_
        )
{

}
*/

#if 0
template <typename AsyncReadStream, typename MutableBufferSequence>
void handle_user_input(
        const boost::system::error_code& ec,
        std::size_t bytes_transferred,
        /*boost::array<uint8_t, STDIN_BUFFER_SIZE> buf&*/
        MutableBufferSequence& stdin_buf,
        AsyncReadStream& stdin_
        )
#endif
#if 0
void handle_user_input(
        const boost::system::error_code& ec,
        std::size_t bytes_transferred,
        boost::asio::streambuf& stdin_buf,
        posix::stream_descriptor& stdin_
        )
{
    if (ec)
    {
        std::cout << "Bollocks" << std::endl;
    }
    else
    {
        std::cout << "Got: " << bytes_transferred << " bytes" << std::endl;
        std::cout << "Received: " << stdin_buf.size() << std::endl;
        async_read(stdin_, stdin_buf, std::bind(handle_user_input, _1, _2,
                    std::cref(stdin_buf), std::cref(stdin_)));
    }
}
#endif

void handle_user_input(
        const boost::system::error_code& ec,
        std::size_t bytes_transferred,
        boost::asio::streambuf& stdin_buf
        )
{
    std::cout << "Bound: " << stdin_buf.size() << std::endl;
    if (ec)
    {
        std::cout << "Bollocks" << std::endl;
    }
    else
    {
        std::cout << "Got: " << bytes_transferred << " bytes" << std::endl;
    }
}

int main()
{
    io_service io_ctx;
    /*boost::array<uint8_t, 100> stdin_buf;*/
    boost::asio::streambuf stdin_buf(STDIN_BUFFER_SIZE);
    /*serial_port ard_port(io_ctx, "/dev/ttyACM0");*/
    posix::stream_descriptor stdin_(io_ctx, dup(STDIN_FILENO));
    posix::stream_descriptor stdout_(io_ctx, dup(STDOUT_FILENO));

    async_read_until(stdin_, stdin_buf, '\n', std::bind(handle_user_input,
                _1, _2, std::cref(stdin_buf)));
    /*async_read_until(stdin_, stdin_buf, '\n', handle_user_input);*/
#if 0
    async_read_until(stdin_, stdin_buf, '\n', std::bind(handle_user_input, _1,
                _2, std::cref(stdin_buf), std::cref(stdin_)));
#endif

    io_ctx.run();
    return 0;
}
