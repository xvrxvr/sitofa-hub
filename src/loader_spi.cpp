#include "common.h"
#include "interf.h"
#include "config.h"

#include <gpiod.hpp>

#include "spi.h"

namespace {

class SPILoader : public Interface::Loader {
    SPI spi;

    gpiod::line l_init, l_done, l_prog;

    bool init() {return l_init.get_value();}
    bool done() {return l_done.get_value();}
    void program(bool val) {l_prog.set_value(val);}

    gpiod::line get_pin(const std::string& name, int dir, bool can_absent=false);

    void spi_init();
public:
    SPILoader();

    virtual void start() override;
    virtual void load(const uint8_t* buffer, size_t size, bool) override {spi.transfer(buffer, NULL, size);}
    virtual void end() override;

    virtual void set_error_reporter(ErrorReporter* er) override {Interface::Loader::set_error_reporter(er); spi.set_error_reporter(er);}
};

#define ERROR(msg)  throw Utils::Exception("SPI Loader: " msg)
#define ERRORF(msg, ...)  throw Utils::Exception(std::format(msg, __VA_ARG__))

gpiod::line SPILoader::get_pin(const std::string& name, int dir = gpiod::line_request::DIRECTION_INPUT, bool can_absent)
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
        if (can_absent) return {};
        ERRORF("Required pin 'GPIO{}' is absent in hardware", pidx);       
    }
    result.request(gpiod::line_request{.request_type = dir}, 1);
    return result;
}

SPILoader::SPILoader()
{
    l_init = get_pin(".pins.init", gpiod::line_request::DIRECTION_INPUT, true);
    l_done = get_pin("GPIO18");
    l_prog = get_pin("GPIO15", gpiod::line_request::DIRECTION_OUTPUT);
}

void SPILoader::start()
{
    if (l_init && !init())  ERROR("INIT_B is low at start");
    program(false);
    Utils::delay_ns(250);
    if (l_init && init()) ERROR("INIT_B not go low after PROGRAM_B pulse");
    program(true);
    if (init_defined())
    {
        Utils::delay_ms(15);
        for(int i=0; !init(); ++i)
        {
            if (i >= 85) ERROR("FPGA not ready");
            Utils::delay_ms(1);
        }
    }
    else
    {
        Utils::delay_ms(100); 
    }
}

void SPILoader::end()
{
    spi.transfer(NULL, NULL, 16);

    if (done()) return;
    if (!l_init) ERROR("Error loading");
    if (!init()) ERROR("CRC  error or Load error");
    ERROR("Loading stall");
}


}

Interface::Loader* Interface::Loader::create_spi_loader() {return new SPILoader;}

