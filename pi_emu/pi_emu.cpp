
#include <iostream>
#include <boost/asio.hpp>
#include <functional>

using namespace boost::asio;
using namespace std::placeholders;

const std::size_t STDIN_BUFFER_SIZE = 1;
struct termios saved_attributes;

/*
void handle_arduino_response(
        const boost::system::error_code& ec,
        std::size_t bytes_transferred,
        posix::stream_descriptor outstream_
        )
{

}
*/

void reset_tty()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &saved_attributes);
}

void set_tty_noncanonical()
{
    /* Check input is from terminal */
    if (!isatty(STDIN_FILENO))
    {
        std::cerr << "Detected non-terminal operation. Attempting to continue. "
            "Note this mode of operation is not tested.";
        return;
    }

    struct termios tattr;

    /* Save the terminal attributes so we can restore them later. */
    tcgetattr(STDIN_FILENO, &saved_attributes);
    atexit(reset_tty);

    /* Set non-canonical terminal mode */
    tcgetattr(STDIN_FILENO, &tattr);
    tattr.c_lflag &= ~(ICANON|ECHO); /* Clear ICANON and ECHO. */
    tattr.c_cc[VMIN] = 1;
    tattr.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &tattr);
}

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
        std::cout << ec << std::endl;
        exit(1);
    }
    else
    {
        std::cout << std::string(buffers_begin(stdin_buf.data()), buffers_begin(stdin_buf.data()) + bytes_transferred) << std::endl;
        stdin_buf.consume(bytes_transferred);
    }
    async_read_until(stdin_, stdin_buf, '\n', std::bind(handle_user_input,
                _1, _2, std::ref(stdin_buf), std::ref(stdin_)));
}

int main()
{
    io_service io_ctx;
    /*boost::array<uint8_t, 100> stdin_buf;*/
    boost::asio::streambuf stdin_buf(STDIN_BUFFER_SIZE);
    /*serial_port ard_port(io_ctx, "/dev/ttyACM0");*/
    posix::stream_descriptor stdin_(io_ctx, ::dup(STDIN_FILENO));
    posix::stream_descriptor stdout_(io_ctx, ::dup(STDOUT_FILENO));
    set_tty_noncanonical();

    async_read(stdin_, stdin_buf, std::bind(handle_user_input,
                _1, _2, std::ref(stdin_buf), std::ref(stdin_)));

    io_ctx.run();
    return 0;
}
