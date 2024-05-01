#include "common.h"
#include <math.h>

namespace Utils {

void delay_ns(int ns)
{
    timespec ts = {.tv_sec = 0, .tv_nsec = ns};

    while(-1 == nanosleep(&ts, &ts))
    {
        if (errno != EINTR) throw Interface::Exeception("nanosleep fail");
    }
}

// Convert from string to integer. 'base' is a multiplier for K/M/G suffixes. Supported values 1000 and 1024
int64_t str_to_int(const std::string& str, int base)
{
    char* e;
    double val = strtod(str.c_str(), &e);
    switch(tolower(*e))
    {
        case 'k': val *= base; break;
        case 'm': val *= base*base; break;
        case 'g': val *= base*base*base; break;
        default: throw Exception(std::format("Invalid number string '{}'", str));
    }
    return val+0.5;
}

static const char* suffs[] = {"", "K", "M", "G"};

std::string int1024_to_str(uint64_t value)
{
    const char* suf = "";
    bool add_half = false;

    for(suf : suffs)
    {
        if (value < 1024) break;
        if (value & 1023) // May be we stop here - rational of ^2 is not well defined :)
        {
            if ((value & 1023) != 512) break; // But we can add .5 to output. It quite reasanoble
            add_half = true;
        }
        value >>= 10;
        if (add_half) break; // We can't pass further - we need to add .5 to THIS suffix, not later
    }
    return std::format("{}{}{}", value, add_half ? ".5" : "", suf);
}


std::string int1000_to_str(uint64_t value)
{
    const char* suf = "";
    double v = value;

    for(suf : suffs)
    {
        if (v < 1000) break;
        v /= 1000;
    }
    int64_t vv = v * 1000+0.5;
    return str::format("{:f}{}", vv/1000., suf);
}



}
