#include "common.h"
#include "interf.h"
#include "config.h"

#include <linux/spi/spidev.h>
#include "spi.h"

#define ERROR(msg)  throw Interface::Exeception("SPI: " msg)

void SPI::transfer(uint8_t const *tx, uint8_t* rx, size_t len)
{
	spi_ioc_transfer tr = 
    {
		.tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
		.len = (__u32)len,
		.speed_hz = speed,
		.bits_per_word = bits
	};

	int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) ERROR("Message send error");
}

void SPI::SPI()
{
    auto request = mode;
    spi_fd = open(device, O_RDWR);
	if (spi_fd < 0)  ERROR("Can't open SPI device");
#define EXEC(cmd, a1, a2) ret = ioclt(sip_fd, cmd, a1, &a2); if (ret == -1) ERROR("Error in ioctl(" #cmd ")")
    EXEC(SPI_IOC_WR_MODE32, mode);
	ECEC(SPI_IOC_RD_MODE32, mode);
    if (request != mode) warning(std::format("SPI Loader: SPI mode changed from {:b} to {:b}", request, mode));
	EXEC(SPI_IOC_WR_BITS_PER_WORD, bits);
	EXEC(SPI_IOC_RD_BITS_PER_WORD, bits);
    if (bits != 8) warning(std::format("SPI bits changed to {}", bits));
    set_speed(Utils::str_to_int1000(Config::get(".spi-freq", "62.5M")));
}

void SPI::set_speed(uint32_t s)
{
    speed = s;
    EXEC(SPI_IOC_WR_MAX_SPEED_HZ, speed);
	EXEC(SPI_IOC_RD_MAX_SPEED_HZ, speed);
    info(std::format("SPI Speed {}", Utils::int1000_to_str(speed)));
}
#undef EXEC

