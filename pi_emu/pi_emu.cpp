
#include <iostream>
#include <boost/asio.hpp>
#include <functional>
#include <boost/array.hpp>

/****************** COMMS DEFS ******************/
#define  HEADER_BYTE   0xFF
#define  CMD_LHS_FWD   0x00
#define  CMD_LHS_BACK  0x01
#define  CMD_RHS_FWD   0x02
#define  CMD_RHS_BACK  0x03
#define  CMD_ACK       0x04
#define  CMD_STALL     0x05

using namespace boost::asio;
using namespace std::placeholders;

const std::size_t ARDUINO_READ_BUFFER_SIZE = 1;
const std::size_t STDIN_BUFFER_SIZE = 1;
struct termios saved_attributes;

#if 0
#pragma GCC diagnostic ignored "-Wmissing-braces"
static std::array<std::array<uint8_t, 7>, 4> instructions =
{
    { HEADER_BYTE, CMD_LHS_FWD, 0x00, 0xFF, 0x00, 0xFF, 0x00 },
    { HEADER_BYTE, CMD_LHS_FWD, 0x00, 0xFF, 0x00, 0xFF, 0x00 },
    { HEADER_BYTE, CMD_LHS_FWD, 0x00, 0xFF, 0x00, 0xFF, 0x00 },
    { HEADER_BYTE, CMD_LHS_FWD, 0x00, 0xFF, 0x00, 0xFF, 0x00 }
};
#endif

static const boost::array<uint8_t, 7> lhs_fwd_fullspeed = 
    {{ HEADER_BYTE, CMD_LHS_FWD, 0x00, 0xFF, 0x00, 0xFF, 0x00 }};
static const boost::array<uint8_t, 7> rhs_fwd_fullspeed = 
    {{ HEADER_BYTE, CMD_RHS_FWD, 0x00, 0xFF, 0x00, 0xFF, 0x00 }};
static const boost::array<uint8_t, 7> lhs_back_fullspeed = 
    {{ HEADER_BYTE, CMD_LHS_BACK, 0x00, 0xFF, 0x00, 0xFF, 0x00 }};
static const boost::array<uint8_t, 7> rhs_back_fullspeed = 
    {{ HEADER_BYTE, CMD_RHS_BACK, 0x00, 0xFF, 0x00, 0xFF, 0x00 }};

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

void handle_arduino_write(
        const boost::system::error_code& ec,
        std::size_t
        )
{
    if (ec)
    {
        std::cout << "Error transferring data to Arduino:" << std::endl;
        std::cout << ec << std::endl;
        std::cout << ec.message() << std::endl;
        exit(3);
    }
    std::cout << "Write to Arduino successful" << std::endl;
}

void handle_arduino_response(
        const boost::system::error_code& ec,
        std::size_t bytes_transferred,
        boost::asio::streambuf& arduino_in_buf,
        serial_port& arduino
        )
{
    if (ec)
    {
        std::cout << "Error in " << __func__ << std::endl;
        std::cout << ec << std::endl;
        std::cout << ec.message() << std::endl;
        exit(2);
    }
    std::cout << "Received from arduino: " << std::string(
            buffers_begin(arduino_in_buf.data()), 
            buffers_begin(arduino_in_buf.data()) + bytes_transferred
            ) << std::endl;
    arduino_in_buf.consume(bytes_transferred);
    async_read(arduino, arduino_in_buf, std::bind(handle_arduino_response,
                _1, _2, std::ref(arduino_in_buf), std::ref(arduino)));
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
        posix::stream_descriptor& stdin_,
        serial_port& arduino
        )
{
    if (ec)
    {
        std::cout << "Error in " << __func__ << std::endl;
        std::cout << ec << std::endl;
        std::cout << ec.message() << std::endl;
        exit(1);
    }
    char instruction = std::string(buffers_begin(stdin_buf.data()), 
            buffers_begin(stdin_buf.data()) + bytes_transferred)[0];
    switch (instruction)
    {
    case '1':
        std::cout << "LHS forward fullspeed" << std::endl;
        async_write(arduino, buffer(lhs_fwd_fullspeed), handle_arduino_write);
        break;
    case '2':
        std::cout << "RHS forward fullspeed" << std::endl;
        async_write(arduino, buffer(rhs_fwd_fullspeed), handle_arduino_write);
        break;
    case '3':
        std::cout << "LHS backward fullspeed" << std::endl;
        async_write(arduino, buffer(lhs_back_fullspeed), handle_arduino_write);
        break;
    case '4':
        std::cout << "RHS backward fullspeed" << std::endl;
        async_write(arduino, buffer(rhs_back_fullspeed), handle_arduino_write);
        break;
    }
    stdin_buf.consume(bytes_transferred);
    async_read(stdin_, stdin_buf, std::bind(handle_user_input, _1, _2,
                std::ref(stdin_buf), std::ref(stdin_), std::ref(arduino)));
}

int main()
{
    io_service io_ctx;
    /*
    try
    {
    */
        streambuf stdin_buf(STDIN_BUFFER_SIZE);
        streambuf arduino_in_buf(ARDUINO_READ_BUFFER_SIZE);

        /* serial_port arduino(arduino_io_ctx, "/dev/ttyACM0"); */
        serial_port arduino(io_ctx, "/dev/ttyACM0");
        serial_port_base::baud_rate baud(115200);
        arduino.set_option(baud);

        set_tty_noncanonical();
        posix::stream_descriptor stdin_(io_ctx, ::dup(STDIN_FILENO));

        async_read(arduino, arduino_in_buf, std::bind(handle_arduino_response,
                    _1, _2, std::ref(arduino_in_buf), std::ref(arduino)));

        async_read(stdin_, stdin_buf, std::bind(handle_user_input, _1, _2,
                    std::ref(stdin_buf), std::ref(stdin_), std::ref(arduino)));
        /*
    }
    catch (const boost::system::error_code& ec)
    {
        std::cout << "Error occurred during setup: " << std::endl;
        std::cout << ec << std::endl;
        std::cout << ec.message() << std::endl;
    }
    catch (const std::exception& ec)
    {
        std::cout << "Error occurred during setup: " << std::endl;
        std::cout << ec.what() << std::endl;
    }
    */

    io_ctx.run();
    return 0;
}

