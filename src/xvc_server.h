#pragma once
#include "interf.h"

class XVC : public Interface::ErrorReportable {
    int srv_socket;
    int clnt_socket
    int stop_fd;
    Interface::JTAG* jtag;
    int buf_size;
    std::unique_ptr<uint8_t[]> tms_buffer, tdi_buffer, tdo_buffer;
    uint8_t* tdo; // Poiter to working part of tdo_buffer. Can be shifted against start of tdo_buffer if JTAG layer requires some byte before 'tdo' buffer
    std::thread worker_thread;
    bool locked = false;

    void worker_main();

        // Socket read/write result
    enum SockResult {
        SR_Ok, // Read success
        SR_Closed, // Socket was closed or error on socket reading/writing. Error will be reported to 'error' interface
        SR_Terminate // Request to terminate XVC server
    };

    // Wait for READIN event on fd.
    SockResult wait(int fd);

    // Read requested number of bytes from clnt_socket
    SockResult read_sock(void* dst, size_t size);

    // Write to socket. Return SR_Ok or SR_Closed (in error case)
    SockResult write_sock(const void* str, size_t size);

    // Read from clnt_socket and match with 'pattern' (2 first symbols will be skipped)
    // If not match - emit error message in 'error reporter' and return SR_Closed
    SockResult match_sock(const char* pattern);

    // Run XVC main loop
    // Return true if it was terminated by 'stop_fd'
    bool xvc_loop();

    // XVC commands
    SockResult xvc_settick();
    SockResult xvc_shift();
    SockResult xvc_getinfo();

public:
    XVC(Interface::JTAG* jtag, int port, int buf_size);
    ~XVC();

    void stop();

    uint32_t set_jtag_freq(uint32_t freq, bool lock);
};
