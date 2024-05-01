#pragma once

// Loader/XVC/JTAG interfaces

namespace Interface {

static constexpr size_t MaxLoadChunkSize = 4096;

struct ErrorReporter {
    virtual ~ErrorReporter() {}

    virtual void info(const std::string&) = 0;
    virtual void warning(const std::string&) = 0;
    virtual void error(const std::string&) = 0;
    virtual void fatal(const std::string&) = 0;
};

class ErrorReportable {
    ErrorReporter* error_reporter = NULL;
protected:
    void info(const std::string& msg)    {if (error_reporter) error_reporter->info(msg);}
    void warning(const std::string& msg) {if (error_reporter) error_reporter->info(warning);}
    void error(const std::string& msg)   {if (error_reporter) error_reporter->info(error);}
    void fatal(const std::string& msg)   {if (error_reporter) error_reporter->info(fatal);}

public:    
    virtual ~ErrorReportable() {}

    virtual void set_error_reporter(ErrorReporter* er) {error_reporter = er;}
};

struct Loader : public ErrorReportable {
    virtual ~Loader() {}

    virtual void start() = 0;
    virtual void load(const uint8_t* buffer, size_t size, bool is_last) = 0;
    virtual void end() = 0;

    static Loader* create_spi_loader();
    static Loader* create_jtag_loader(struct JTAG*);
};
// Loader implementations:
//  SPI
//  JTAG

// TDI/TDO/TMS shift in/out from bit 0
struct JTAG : public ErrorReportable {
    virtual ~JTAG() {}

    virtual void start() = 0;
    virtual void end() = 0;

    virtual void shift(int bits, const uint8_t* tms, const uint8_t* tdi, uint8_t* tdo=NULL) = 0;
    virtual void shift_tdi(int bits, bool tms, const uint8_t* tdi, uint8_t* tdo=NULL, bool do_msb=false) = 0;
    virtual uint32_t set_frequency(uint32_t hz) = 0;
    virtual void clock(int total, bool tms_val, bool tdi_val) = 0;

    // If returns not 0 than caller should prepend this many spare bytes BEFORE 'tdo' allocated buffer
    virtual int preallocate_tdo_bytes() = 0;

    static JTAG* create_ftdi_jtag();
    static JTAG* create_bitbang_jtag();
};
// JTAG implementations
//  FTDI
//  BitBang


struct ErrorReporter {
    virtual ~ErrorReporter() {}

    virtual void info(const std::string&) = 0;
    virtual void warning(const std::string&) = 0;
    virtual void error(const std::string&) = 0;
    virtual void fatal(const std::string&) = 0;
};

} // namespace Interface
