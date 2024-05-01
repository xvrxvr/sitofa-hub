#include "common.h"
#include "interf.h"
#include "config.h"

#include <gpiod.hpp>

namespace {

class BitBangJTAG : public Interf::JTAG {
    gpiod::line l_tms, l_tck, l_tdi, l_tdo;

    gpiod::line get_pin(const std::string& name, int value=0, int dir = gpiod::line_request::DIRECTION_OUTPUT);

    void shift_any(int bits, const uint8_t* tms, uint8_t tms_scalar, const uint8_t* tdi, uint8_t* tdo, bool do_msb);
    uint8_t ticks(uint8_t tms, uint8_t tdi, int bits, bool do_msb);

public:
    BitBangJTAG();

    virtual void start() override {}
    virtual void end() override {}

    virtual void shift(int bits, const uint8_t* tms, const uint8_t* tdi, uint8_t* tdo=NULL) override {shift_any(bits, tms, false, tdi, tdo, false);}
    virtual void shift_tdi(int bits, bool tms, const uint8_t* tdi, uint8_t* tdo=NULL, bool do_msb=false) override {shift_any(bits, NULL, tms ? 0xFF : 0, tdi, tdo, do_msb);}
    virtual uint32_t set_frequency(uint32_t hz) override {return hz;}
    virtual void clock(int total, bool tms_val, bool tdi_val) override;

    virtual int preallocate_tdo_bytes() override {return 0;}
};

#define ERROR(msg)  throw Utils::Exception("BitBang JTAG: " msg)
#define ERRORF(msg, ...)  throw Utils::Exception(std::format(msg, __VA_ARG__))


BitBangJTAG::BitBangJTAG()
{
    l_tms = get_pin(".pins.TMS", 1);
    l_tck = get_pin(".pins.TCK");
    l_tdo = get_pin(".pins.TDO");
    l_tdi = get_pin(".pins.TDI", 0, gpiod::line_request::DIRECTION_INPUT);
}

gpiod::line BitBangJTAG::get_pin(const std::string& name, int value, int dir)
{
    int pidx = Config::pin(name);
    if (pidx == -1)
    {
        if (can_absent) return {};
        ERRORF("Required pin '{}' is absent in config", name);
    }
    gpiod::line result = gpiod::find_line(std::print("GPIO{}", pidx).c_str());
    if (!result)
    {
        ERRORF("Required pin 'GPIO{}' is absent in hardware", pidx);       
    }
    result.request(gpiod::line_request{.request_type = dir}, value);
    return result;
}

void BitBangJTAG::shift_any(int bits, const uint8_t* tms, uint8_t tms_scalar, const uint8_t* tdi, uint8_t* tdo, bool do_msb)
{
    while(bits)
    {
        int l = std::min(8, bits);
        uint8_t data = ticks(tms ? *tms++ : tms_scalar, *tdi++, l, do_msb);
        if (tdo) *tdo++ = data;
        bits -= l;
    }
}

uint8_t BitBangJTAG::ticks(uint8_t tms, uint8_t tdi, int bits, bool do_msb)
{
    uint8_t result = 0;
    uint8_t mask = 1;
    while(bits--)
    {
        if (do_msb) l_tdi.set_value(tdi>>7);
        else l_tdi.set_value(tdi&1);
        l_tms.set_value(tms&1);
        if (l_tdo.get_value()) result |= mask;
        l_tck.set_value(1);
        mask <<= 1;
        tms >>= 1;
        if (do_msb) tdi <<= 1;
        else tdi >>= 1;
        l_tck.set_value(0);
    }
    return result;
}

void BitBangJTAG::clock(int total, bool tms_val, bool tdi_val)
{
    l_tms.set_value(tms_val);
    l_tdi.set_value(tdi_val);
    while(total--)
    {
        l_tck.set_value(1);
        l_tck.set_value(0);
    }
}

}

Interf::JTAG* Interf::JTAG::create_bitbang_jtag() {return new BitBangJTAG;}

