#include "common.h"
#include "interf.h"

#include <sys/eventfd.h>
#include <poll.h>

#include "xvc_server.h"

#define ERROR(msg)  throw Utils::Exeception("XVC Server: " msg)
#define ERRORF(msg, ...)  throw Utils::Exeception(std::format(msg, __VA_ARG__))

XVC::XVC(Interface::JTAG* jtag, int port, int buf_size) 
    : jtag(jtag), 
      buf_size(buf_size), 
      tdi_buffer(new uint8_t[buf_size]), 
      tms_buffer(new uint8_t[buf_size]), 
      tdo_buffer(new uint8_t[buf_size+jtag->preallocate_tdo_bytes()])
{
    int o = 1;
    sockaddr_in addr{.sin_family=AF_INET, .sin_addr=INADDR_ANY, .sin_port = htons(port) };
    srv_socket = -1;

    tdo = tdo_buffer.get() + jtag->preallocate_tdo_bytes();
    stop_fd = eventfd(0, EFD_NONBLOCK);
    if (stop_fd == -1) ERROR("Can't create Stop event");

    srv_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (srv_socket < 0) ERROR("socket");
    if (setsockopt(srv_socket, SOL_SOCKET, SO_REUSEADDR, &o, sizeof(o)) < 0) ERROR("setsockopt");
    if (bind(srv_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) ERROR("bind");
    if (listen(srv_socket, 1) < 0) ERROR("listen");

    worker_thread = std::thread([this]() {worker_main();});
}

XVC::~XVC()
{
    if (stop_fd != -1) {stop(); close(stop_fd);}
    if (srv_socket != -1) close(srv_socket);
}

void XVC::stop()
{
    char one = 1;
    write(stop_fd, &one, 1);
    if (worker_thread.joinable()) worker_thread.join();
}


// Wait for READIN event on fd.
XVC::SockResult XVC::wait(int fd)
{
    pollfd pp[2] = {
        {.fd = fd, .events = POLLIN},
        {.fd = stop_fd, .events = POLLIN}
    };
    while (-1 == poll(pp, 2, -1))
    {
        if (errno != EINTR) ERROR("poll");
    }
    if (pp[1].revents) return SR_Terminate;
    if (pp[0].revents & POLLHUP) return SR_Closed;
    if (pp[0].revents & POLLERR) ERROR("Socket error");
    return SR_Ok;
}

// Read requested number of bytes from clnt_socket
XVC::SockResult XVC::read_sock(void* dst, size_t size)
{
    uint8_t* d = (uint8_t*)d;
    while(size)
    {
        if (auto err = wait(clnt_socket)) return err;
        int bytes_available=0;
        ioctl(clnt_socket,FIONREAD,&bytes_available);
        if (!bytes_available) continue;
        if (bytes_available > size) bytes_available = size;
        auto res = read(clnt_socket, d, bytes_available);
        if (res != bytes_available) ERRORF("Socker read - request {}, real {}", bytes_available, res);
        d += bytes_available;
        size -= bytes_available;
    }
}

// Write to socket. Return SR_Ok or SR_Closed (in error case)
XVC::SockResult XVC::write_sock(const void* str, size_t size)
{
    const uint8_t* s = (const uint8_t*)str;
    while(size)
    {
        auto result = send(clnt_socket, s, size, MSG_NOSIGNAL);
        if (result >= 0)
        {
            s += result;
            size -= result;
        }
        else
        {
            switch(errno)
            {
                case ECONNRESET: case EPIPE: return SR_Closed;
                case EINTR: case ENOBUFS: break;
                default: ERROR("Socked send error");
            }
        }
    }
    return SR_Ok;
}

// Read from clnt_socket and match with 'pattern' (2 first symbols will be skipped)
// If not match - emit error message in 'error reporter' and return SR_Closed
XVC::SockResult XVC::match_sock(const char* pattern)
{
    char msg[64];
    auto len = strlen(pattern);
    assert(len < sizeof(msg) && len > 2);
    if (auto err = read_sock(msg, len-2)) return err;
    if (memcmp(pattern+2, msg, len-2) == 0) return SR_Ok;
    msg[len-2] = 0;
    error(std::format("Unexpected command got '{}{}{}' (expected '{}')", pattern[0], pattern[1], msg));
    return SR_Close; 
}


void XVC::worker_main()
{
    try
    {
        info("XVC Server Started");
        for(;;)
        {
            if (wait(srv_socket)) break;
            clnt_socket = accept(srv_socket, NULL, NULL);
            if (clnt_socket < 0) ERROR("accept");

            info("XVC Connected");
            bool terminate = xvc_loop();
            close(clnt_socket);
            if (terminate) break;
            info("XVC Disconnected");
        }
        info("XVC Server Terminated");
    }
    catch(std::exception exp)
    {
        error(std::format("XVC Server was terminated by exception: {}", exp.what()));
    }
}

// Run XVC main loop
// Return true if it was terminated by 'stop_fd'
bool XVC::xvc_loop()
{
    try 
    {
        jtag->start();
        for(;;)
        {
            char c[3]={};
#define E(cmd) if (auto res = cmd) return res == SR_Terminate
            E(read_sock(c, 2));
#define V2(c1, c2) ((c1) | ((c2) << 8))
#define EXPECT(msg) E(match_sock(msg))
            switch(V1(c[0], c[1]))
            {
                case V2('s', 'e'): EXPECT("settck:");  E(xvc_settick()); break;
                case V2('s', 'h'): EXPECT("shift:");   E(xvc_shift());   break;
                case V2('g', 'e'): EXPECT("getinfo:"); E(xvc_getinfo()); break;
                default: error(std::format("XVC: Unexpected command '{}'", c)); return false; 
            }
#undef EXPECT
#undef V2
#undef E
        }
        jtag->end();
    } 
    catch (std::exception exp) 
    {
        error(std::format("XVC Server session was terminated by exception: {}", exp.what()));
    }
    return false;
}

XVC::SockResult XVC::xvc_getinfo()
{
    auto msg = std::format("xvcServer_v1.0:{}\n", buf_size);
    return write_sock(msg.c_str(), msg.size());
}

XVC::SockResult XVC::xvc_settick()
{
    uint32_t freq;
    if (auto err = read_sock(&freq, sizeof(freq))) return err;
    if (locked)
    {
        info("XVC: Set JTAG Freq ignored");
    }
    else
    {
        freq = 1000000000 / freq;
        freq = jtag->set_frequency(freq);
        freq = 1000000000 / freq;
    }
    return write_sock(&freq, sizeof(freq));
}

uint32_t XVC::set_jtag_freq(uint32_t freq, bool lock)
{
    locked = lock;
    freq = jtag->set_frequency(freq);
    info(std::format("XVC JTAG: Freq set to {}{}", Utils::int1000_to_str(freq), lock ? " [Locked]" : ""));
    return freq;
}


XVC::SockResult XVC::xvc_shift()
{
    uint32_t len;
    if (auto err = read_socket(&len, sizeof(len))) return err;
    uint32_t bytes = (len+7)/8;
    if (bytes > buf_size) {error(std:;format("XVC shift: Requested to read {} bytes, max is {}", bytes, buf_size)); return SR_Closed;}
    if (auto err = read_socket(tms_buffer.get(), bytes)) return err;
    if (auto err = read_socket(tdi_buffer.get(), bytes)) return err;
    jtag->shift(len, tms_buffer.get(), tdi_buffer.get(), tdo);
    return write_socket(tdo, len);
}
