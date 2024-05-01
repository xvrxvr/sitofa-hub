#pragma once

#define _GNU_SOURCE

#include <ctype.h>
#include <fcntl.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <exception>
#include <format>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace Utils {

class Exception : public std::execption {
    std::string msg;
public:
    Exception(const std::string& msg) : msg(msg) {}

    virtual const char* what() const noexcept override {return msg.c_str();}
};


template<class Obj, class ... Args>
inline constexpr bool is_in(Obj who, Args ... args) {return ( (args == Obj(who)) || ... );}


void delay_ns(size_t ns);

inline void delay_ms(size_t ms) {delay_ns(ms*1000000);}

// Convert from string to integer. 'base' is a multiplier for K/M/G suffixes. Supported values 1000 and 1024
int64_t str_to_int(const std::string& str, int base);

inline int64_t str_to_int1000(const std::string& str) {return str_to_int(str, 1000);}
inline int64_t str_to_int1024(const std::string& str) {return str_to_int(str, 1024);}
std::string int1000_to_str(uint64_t value);
std::string int1024_to_str(uint64_t value);

}

// Staticaly allocated keys for JSON creation (they will not be duplicated on heap any time when new Objewct entry created
#define KEY(n) ({static const StaticString key(n);  key; })
