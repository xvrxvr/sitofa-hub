#pragma once

#include "interf.h"

class Peer : public Interf::ErrorReporter {
    int srv_fd=-1;
    int clnt_fd=-1;
    int stop_fd=-1;
    bool loop_break_request=false;

    std::unique_ptr<char[]> buffer;
    size_t buffer_size=0;

    // Return true if stop request detected or socket closed
    // Raise exception on poll or socket error
    // timeout in milliseconds
    bool wait_sock(int sock_fd, int timeout);

    // Read data from clnt_fd
    // Returns size of data read
    // On error raise Exception
    size_t read_sock();

public:
    Peer(const std::string& name);
    ~Peer();

    void set_stop_fd(int stop_fd) {this->stop_fd=stop_fd;}

    // Wait for cliet connection, than cycle inside and exit when client close connection
    // On fatal errors raise Exception (and disconnect client)
    // Non fatal error passed to ErrorReporter
    void connect_client(bool with_timeout=true);

    // Break loop in 'connect_client' and exit
    void exit_from_client() {loop_break_request=true;}

    void send_data(const std::string& json, int fd=-1);
    void send_data(uint64_t data) {send_data(std::string((char*)&data, sizeof(data)));

    Peer& operator<<(uint64_t data) {send_data(data); return *this;}
    Peer& operator<<(const Json::Value& data); // JSON write

    virtual void on_json_rcv(const Json::Value&) = 0;
    virtual void on_data_rcv(uint64_t) = 0;
};
