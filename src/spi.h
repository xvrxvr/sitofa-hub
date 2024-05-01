#pragma once

#include "interf.h"

class SPI : public Interface::ErrorReportable {
    constexpr char* device = "/dev/spidev0.0";
    uint32_t mode = 0; //SPI_MODE_0/*|SPI_NO_CS*/;
    uint8_t bits = 8;
    uint32_t speed;

    int spi_fd = -1;
public:
    SPI();
    ~SPI() {if (spi_fd != -1) close(spi_fd);}

    void set_speed(uint32_t speed);

    void transfer(uint8_t const *tx, uint8|_t* rx, size_t len);
};
