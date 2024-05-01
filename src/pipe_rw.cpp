#include "common.h"
#include "pipe_rw.h"

#include <poll.h>
#include <sstream>

static constexpr int open_timeout = 5; // 5 seconds for open

#define ERROR(msg)  throw Interface::Exeception("Unix Socket: " msg)
#define ERRORF(msg, ...)  throw Interface::Exeception(std::format(msg, __VA_ARG__))

Peer::Peer(const std::string& name)
{
    sockaddr_un addr {.sun_family = AF_UNIX};
    memcpy(addr.sun_path, name.c_str(), std::min(sizeof(addr.sun_path), name.size()));
    srv_fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if (srv_fd < 0) ERROR("socket");
    if (bind(srv_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) ERROR("bind");
    if (listen(srv_fd, 1) < 0) ERROR("listen");
}

Peer::~Peer()
{
    if (clnt_fd != -1) close(clnt_fd);
    if (srv_fd != -1) close(srv_fd);
}

bool Peer::wait_sock(int sock_fd, int timeout)
{
    pollfd pp[2] = {
        {.fd = sock_fd, .events = POLLIN},
        {.fd = stop_fd, .events = POLLIN}
    };
    while (-1 == poll(pp, stop_fd == -1 ? 1 : 2, timeout))
    {
        if (errno != EINTR) ERROR("poll");
    }
    if (pp[1].revents) return true;
    if (pp[0].revents & POLLHUP) return true;
    if (pp[0].revents & POLLERR) ERROR("Socket error");
    return false;
}

// Wait for cliet connection, than cycle inside and exit when client close connection
// On fatal errors raise Exception (and disconnect client)
// Non fatal error passed to ErrorReporter
void Peer::connect_client(bool with_timeout)
{
    if (wait_sock(srv_fd, with_timeout ? open_timeout*1000 : -1)) return;
    clnt_fd = accept(srv_fd, NULL, NULL);
    if (clnt_socket < 0) ERROR("accept");
    while(!loop_break_request)
    {
        if (wait_sock(clnt_fd, -1)) break;
        auto data_size = read_sock();
        if (!data_size) break; // EOF - socket was closed from other side
        if (buffer[0] == '{') 
        {
            Json::Value val;
            std::string errs;
            std::unique_ptr<Json::CharReader> reader(Json::CharReaderBuilder().newCharReader());

            if (reader->parse(buffer.get(), data_size+buffer.get(), &val, &errs)) on_json_rcv(val); else
            {
                // Recieved string is not valid JSON. Write error and send JSON with error as answer.
                // User will not recieved anything in this case
                error(std::format("JSON error in input string: {}", errs));
                Json::Value val;
                val[KEY("result")] = "error";
                val[KEY("message")] = errs;
                *this << val;
            }
        }
        else
        {
            if (data_size != sizeof(uint64_t)) error(std::format("Wrongly sized binary data recieved from client - {} bytes, should be 8", data_size));
            else on_data_rcv(*(uint64_t*)buffer.get());
        }
    }
    if (clnt_fd != -1) {close(clnt_fd); clnt_fd=-1;}
}

Peer& Peer::operator<<(const Json::Value& data)
{
    Json::FastWriter writer;
    writer.omitEndingLineFeed();
    send_data(writer.write(data));
    return *this;
}

size_t Peer::read_sock()
{
    int bytes_available=0;
    ioctl(clnt_fd,FIONREAD,&bytes_available);
    if (!bytes_available) return {0, -1};
    if (buffer_size < bytes_available)
    {
        buffer_size = std::max(512, bytes_available + bytes_available/2);
        buffer.reset(new char[buffer_size]);
    }
    auto res = read(clnt_fd, buffer.get(), bytes_available);
    if (res != bytes_available) ERRORF("Socker read - request {}, real {}", bytes_available, res);
    return bytes_available;
}

void Peer::send_data(const std::string& json, int fd)
{
    iovec msg_iov{
        .iov_base = json.c_str(),
        .iov_len = json.size()
    };
    union {
        char buf[CMSG_SPACE(sizeof(ind))];
        struct cmsghdr align;
    };
    msghdr mhdr{
        .msg_iov=&msg_iov,
        .msg_iovlen=1,
    };
    if (fd != -1)
    {
        mhdr.msg_control = buf;
        mhdr.msg_controllen = sizeof(buf);
        auto cmsg = CMSG_FIRSTHDR(&msg);
        cmsg->cmsg_level = SOL_SOCKET;
        cmsg->cmsg_type = SCM_RIGHTS;
        cmsg->cmsg_len = CMSG_LEN(sizeof(int));
        memcpy(CMSG_DATA(cmsg), &fd, sizeof(int));
    }
    auto res = sendmsg(clnt_fd, &mhdr, MSG_NOSIGNAL);
    if (res == -1)
    {
        switch(errno)
        {
            case EPIPE: case ECONNRESET: // Client close its socket
                error("UNIX Socket Send: Client close its peer, exiting"); 
                loop_break_request=true;
                return;
            case EINTR: send_data(json, fd); return; // Run again
            default: ERROR("Send failed");
        }
    }
    if (res != json.size())
    {
        ERRORF("Socket write: Trimmed - request {}, sent {}", json.size(), res);
    }
}

