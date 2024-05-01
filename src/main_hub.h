#pragma once

#include <json.h>

#include "interfaces.h"

enum JtagType {
    JT_Auto,
    JT_Disable,
    JT_FTDI,
    JT_BitBang
};


class Hub {
public:
    Hub();
    ~Hub();

    void add_error_reporter(Interface::ErrorReporter*);
    void remove_error_reporter(Interface::ErrorReporter*);

    bool set_jtag_type(JtagType);
    int set_jtag_freq(int freq, bool lock);

    bool load_bin_file(const std::string& fname, bool lock);
    bool load_pack_file(const std::string& fname);
    Json::Value get_project_data();
    void unload_project(); // Unregister all HUB clients, reset FFGA

    void enable_xvc(bool);

    ///// HUB interface

    // Register Client (by name). Push/Event pipes, memory
    // Unregister Client
};
