#include "common.h"
#include "interf.h"
#include "config.h"

#include <libusb-1.0/libusb.h>

static_assert(LIBUSB_API_VERSION >= 0x01000109, "Expected libusb of 1.0.26 or later version"); // Debian bookworm last package

namespace {

#define ERROR(msg)  throw Interface::Exeception("FTDI JTAG: " msg)
#define ERRORF(msg, ...)  throw Interface::Exeception(std::format(msg, __VA_ARG__))

class LibUsb {
    libusb_context* usb = NULL;
    libusb_device_handle* handle = NULL;
    int dev_idx;    // usblib index of device
    int ftdi_index; // 1 for 'A', 2 for 'B'

    struct EndPoint {
        int index=0;
        size_t size=0;

        void operator =(const libusb_endpoint_descriptor& ep)
        {
            if (index) ERROR("Too many endpoints");
            index=ep.bEndpointAddress;
            size=ep.wMaxPacketSize;
        }
        void check()
        {
            if (!index) ERROR("Endpoint is not assigned");
        }
    } in_endp, out_endp;

    void find_endpoints(const libusb_interface_descriptor &iface_desc);
    int find_device(libusb_device **dev_list, int total);

public:
    LibUsb();
    ~LibUsb();                          

    void control(int request, int value);
    void write(const void* data, size_t size);
    void read(void* data, size_t size);

    size_t in_endp_size() const {return in_endp.size;}
};

#define E(cmd) res = cmd; if (res<0) ERRORF(#cmd " failed: {}", libusb_strerror(res))

LibUsb::LibUsb()
{
    libusb_device **dev_list;
    int ret;

    libusb_init(&usb);

    ssize_t total = libusb_get_device_list(usb, &dev_list);
    if (total < 0) ERRORF("libusb_get_device_list failed: {}", libusb_strerror(int(total))); 

    dev_idx = find_device(dev_list, total);
    libusb_free_device_list(dev_list, 1);

    if (dev_idx == -1) ERROR("FTDI device not found");
    if (libusb_set_auto_detach_kernel_driver(handle, 1))
    {
        E(libusb_kernel_driver_active(handle, dev_idx));
        if (res) 
        {
            E(libusb_detach_kernel_driver(handle, dev_idx));
        }
    }
    E(libusb_claim_interface(handle, dev_idx));
}

LibUsb::~LibUsb()
{
    if (handle) {libusb_release_interface(handle, dev_idx); libusb_close(handle);}
    if (usb) libusb_exit(usb);
}

void LibUsb::find_endpoints(const libusb_interface_descriptor &iface_desc)
{
    for (int idx = 0; idx < iface_desc.bNumEndpoints; idx++) 
    {
        const libusb_endpoint_descriptor &ep = iface_desc.endpoint[idx];
        if ((ep.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_BULK) 
        {
            ((ep.bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) == IBUSB_ENDPOINT_IN ? in_endp : out_endp) = ep;
        }
    }
    in_endp.check(); 
    out_endp.check();
}

int LibUsb::find_device(libusb_device **dev_list, int total)
{
    int res;

    auto cfg_vid = Config::get(".FTDI.vid", 0x0403);
    auto cfg_pid = Config::get(".FTDI.pid", -1);
    auto cfg_serial = Config::get(".FTDI.serial", "");
    uint32_t cfg_index = (Config::get(".FTDI.index", "A")+"A")[0]-'A';

    if (cfg_index > 1) ERRORF("Wrong FTDI index '{}', shoud be A or B", char('A' + cfg_index));

    ftdi_index = cfg_index + 1;

    for (int i = 0 ; i < total ; i++) 
    {
        libusb_device *dev = dev_list[i];
        libusb_device_descriptor desc;
        libusb_config_descriptor *config;

        E(libusb_get_device_descriptor(dev, &desc));
        if (desc.bDeviceClass != LIBUSB_CLASS_PER_INTERFACE) continue;

        if (cfg_pid== -1 ? !Utils::is_in(desc.idProduct, 0x6010, 0x6011, 0x6014) : desc.idProduct != cfg_pid) continue;
        if (cfg_vid != desc.idVendor) continue;
        if ((libusb_get_active_config_descriptor(dev, &config) < 0)
            && (libusb_get_config_descriptor(dev, 0, &config) < 0)) 
        {
            warning(std::format("Can't get vendor {:04X} product {:04X} configuration.", desc.idVendor, desc.idProduct));
            continue;
        }
        if (!config) continue;
        std::unique_ptr<libusb_config_descriptor, void (libusb_config_descriptor*)> config_free(config, libusb_free_config_descriptor);
        if (config->bNumInterfaces > cfg_index) 
        {
            E(libusb_open(dev, &handle));

            const libusb_interface &iface = config->interface[cfg_index];
            const libusb_interface_descriptor &iface_desc =iface.altsetting[0];

            if (!cfg_serial.empty())
            {
                std::vector<char> buf(cfg_serial.size());
                ssize_t len = libusb_get_string_descriptor_ascii(handle, iface_desc.iSerialNumber, buf.data(), buf.size());
                if (len != cfg_serial.size() || memcmp(cfg_serial.c_str(), buf.data(), len) != 0) 
                {
                    libusb_close(handle);
                    handle = NULL;
                    continue;
                }
            }
            get_endpoints(iface_desc);
            return iface_desc.bInterfaceNumber;
        }
    }
    return -1;
}

void LibUsb::control(int request, int value)
{
    int res;
    E(libusb_control_transfer(handle, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT, request, value, ftdi_index, NULL, 0, 1000));
}

void LibUsb::write(const void* data, size_t size)
{
    const uint8_t* p = (const uint8_t*)data;
    int res;

    while(size) 
    {
        int s = std::min<int>(size, out_endp.size);
        E(libusb_bulk_transfer(handle, out_endp.index, p, s, &s, 10000));
        size -= s;
        p += s;
    }

}

void LibUsb::read(void* data, size_t size)
{
    uint8_t* d = ((uint8_t*) data) - 2;
    int res; // for E macro

    while(size) 
    {
        int s = std::min<int>(size+2, in_endp.size);
        uint8_t sv1 = d[0], sv2 = d[1]; // Save 2 bytes from previous pack - it will be overriden by FTDI status bytes
        E(libusb_bulk_transfer(handle, in_endp.index, d, s, &s, 5000));
        d[0] = sv1; d[1] = sv2; // Restore 2 last bytes in previous pack
        if (s <= 2) continue;
       
        s -= 2;  // Skip FTDI status bytes

        size -= s;
        d += s;
    }
}

////////////////////////////////////////////////////////////////////////
// libusb libusb_control_transfer.bRequest
#define BREQ_RESET          0x00
#define BREQ_SET_LATENCY    0x09
#define BREQ_SET_BITMODE    0x0B

// libusb libusb_control_transfer.wValue for assorted libusb_control_transfer.bRequest values
#define WVAL_RESET_RESET        0x00
#define WVAL_RESET_PURGE_RX     0x01
#define WVAL_RESET_PURGE_TX     0x02
#define WVAL_SET_BITMODE_MPSSE (0x0200 | FTDI_PIN_TCK | FTDI_PIN_TDI | FTDI_PIN_TMS)

// FTDI commands (first byte of bulk write transfer)
#define FTDI_MPSSE_BIT_WRITE_TMS                0x40
#define FTDI_MPSSE_BIT_READ_DATA                0x20
#define FTDI_MPSSE_BIT_WRITE_DATA               0x10
#define FTDI_MPSSE_BIT_LSB_FIRST                0x08
#define FTDI_MPSSE_BIT_READ_ON_FALLING_EDGE     0x04
#define FTDI_MPSSE_BIT_BIT_MODE                 0x02
#define FTDI_MPSSE_BIT_WRITE_ON_FALLING_EDGE    0x01

#define FTDI_MPSSE_XFER_COMMON    (FTDI_MPSSE_BIT_LSB_FIRST | FTDI_MPSSE_BIT_WRITE_ON_FALLING_EDGE)

#define FTDI_SET_LOW_BYTE           0x80
#define FTDI_SET_HIGH_BYTE          0x82
#define FTDI_DISABLE_LOOPBACK       0x85
#define FTDI_SET_TCK_DIVISOR        0x86
#define FTDI_DISABLE_TCK_PRESCALER  0x8A
#define FTDI_DISABLE_3_PHASE_CLOCK  0x8D
#define FTDI_ACK_BAD_COMMAND        0xFA
#define FTDI_MPSSE_CLOCK_BITS       0x8E
#define FTDI_MPSSE_CLOCK_BYTES      0x8F

// FTDI I/O pin bits
#define FTDI_PIN_TCK    0x1
#define FTDI_PIN_TDI    0x2
#define FTDI_PIN_TDO    0x4
#define FTDI_PIN_TMS    0x8

#define FTDI_CLOCK_RATE     60000000

// Managemnt of TDI/TMS data spans
// Split them in bit/byte runs and handle runs

enum SpanType {
    ST_None, // No more spans
    ST_Bit, // Bit span (up to 1 byte)
    ST_Byte // Byte span
};

enum BitSpanType {
    BiST_None, // No more bit spans
    BiST_TMS,
    BiST_TDI
};

struct BitSpanData {
    bool static_val;
    bool need_tdo;
    uint8_t len;
    uint8_t shift;
};

enum ByteSpanType {
    BtST_None, // No more spans
    BtST_Data, // Data span
    BtST_CLK   // Only clk
};

struct ByteSpanData {
    bool h_tms;       // TMS value to hold
    bool h_tdi;       // TDI value to hold (for BtST_CLK span type)
    size_t count;   // How many bytes/bits
    uint8_t* tdi;   // Data to send (BtST_DATA only)
    uint8_t* tdo;   // Optional (f not NULL) buffer to TDO data (BtST_DATA only)
};

class SpanMgr {
    uint8_t* tms;
    uint8_t* tdi;
    uint8_t* tdo;
    size_t length;

    // Bit Span accumulation
    uint8_t bi_acc;
    uint8_t bi_out_shift;
    uint8_t bi_tdi, bi_tms;
    uint8_t bi_mask;

    // Byte span accumulation
    uint8_t  by_tms;
    uint8_t* by_tdi;
    uint8_t* by_tdo;

    // Common
    uint16_t acc_length;

    void setup_bit() {bi_acc=bi_out_shift=0; acc_length = std::min<int>(8, length); bi_tdi = *tdi++; bi_tms = *tms++;}
    void setup_byte();

    ByteSpanType detect_byte_span_segment(int& length);

public:
    SpanMgr(size_t len, uint8_t* tms, uint8_t* tdi, uint8_t* tdo=NULL) : tms(tms), tdi(tdi), tdo(tdo), length( len) {}

    SpanType has_span();

    BitSpanType get_bit_span(BitSpanData&);
    void put_bit_span_data(uint8_t val);

    ByteSpanType get_byte_span(ByteSpanData&);
};

SpanType SpanMgr::has_span()
{
    if (!length) return ST_None;
    if (length < 16) {setup_bit(); return ST_Bit;}
    if (tms[0] == tms[1] && Utils::is_in(tms[0], 0, 0xFF)) {setup_byte(); return ST_Byte;}
    setup_bit();
    return ST_Bit;
}

void SpanMgr::setup_byte()
{
    assert(length >= 8);
    by_tms = tms[0];
    assert(Utils::is_in(by_tms, 0, 0xFF));

    size_t max_l = length / 8;
    int l;
    for(l=0; l<max_l; ++l) if (tms[l] != by_tms) break;

    tms += l;
    by_tdi = tdi; tdi += l;
    if (tdo) {by_tdo = tdo; tdo += l;}

    acc_length = l;
    length -= l*8;
}

inline constexpr int mask(int bit) {return (1<<bit)-1;}

static int same_bit_len(uint8_t data, int length)
{
    uint8_t mask = mask(length);
    data &= mask;
    if (data == 0 || data == mask) return length; // All 'data' is 0 or 1 - full length constant
    if (data & 1) data ^= mask; // Make same bits run always zero
    return std::min<int>(length, __builtin_ctz(data));
}

BitSpanType SpanMgr::get_bit_span(BitSpanData& data)
{
    if (!acc_length) // Done - push data, if need
    {
        if (tdo) *tdo++ = bi_acc;
        return BiST_None;
    }
    data.need_tdo = tdo != NULL;
    int tdi_same_length = same_bit_len(bi_tdi, acc_length);
    int tms_same_length = same_bit_len(bi_tms, acc_length);

    BitSpanType result;
    uint8_t len;

    if (tms_same_length >= tdi_same_length) // Use TDI run (TMS is constant)
    {
        len = tms_same_length;
        data.static_val = bi_tms & 1;
        data.shift = bi_tdi;
        result = BiST_TDI;
    }
    else
    {
        len = std::min<uint_8>(7, tdi_same_length);
        data.static_val = bi_tdi & 1;
        data.shift = bi_tms;
        result = BiST_TMS;
    }
    data.shift &= (bi_mask = mask(len));
    if (result == BiST_TMS && data.static_val) data.shift |= 0x80;
    bi_tms >>= len;
    bi_tdi >>= len;
    acc_length -= len;
    bi_out_shift += len;
    data.len = len;
    return result;
}

void SpanMgr::put_bit_span_data(uint8_t val)
{
    bi_acc |= (val & bi_mask) << bi_out_shift;
}

ByteSpanType SpanMgr::get_byte_span(ByteSpanData& data)
{
    if (!acc_length) return BtST_None;

    data.h_tms = by_tms;
    data.h_tdi = by_tdi[0]&1;
    data.tdi = by_tdi;
    data.tdo = by_tdo;

    if (tdo)
    {
        data.count = acc_length;
        acc_length = 0;
        return BtST_Data;
    }

    int result_len;
    auto result = detect_byte_span_segment(result_len);

    by_tms += result_len;
    by_tdi += result_len;

    acc_length -= result_len;

    if (result == BtST_CLK) // Scale clock count and try to append tail of buffer to clk count
    {
        result_len *= 8;
        if (length < 8) // Just 1 not full data to send in next run - try to append
        {
            int msk = mask(length);
            uint8_t m_tms = data.h_tms ? 0xFF : 0;
            uint8_t m_tdi = data.h_tdi ? 0xFF : 0;
            if ( !(((tms[0] ^ m_tms) | (tdi[0] ^ m_tdi)) & msk) )  // All mactch - can add it to our span
            {
                result_len += length;
                length = 0;
            }
        }
    }
    data.count = result_len;
    return result;
}

ByteSpanType SpanMgr::detect_byte_span_segment(int& ret_length)
{
    et_length = acc_length;
    if (acc_length < 4) return BtST_Data;
    if (Utils::is_in(bi_tms[0], 0, 0xFF)) // CLK span
    {
        for(int l=0; l<acc_length; ++l)
        {
            if (!Utils::is_in(bi_tms[0], 0, 0xFF)) {ret_length = l; return BtST_CLK;}
        }
        return BtST_CLK;
    }
    // Data run. Scan for stable TDI data for at least 3 slots
    for(int idx=0, last_data=0, stable_tdi=0; idx<acc_length; ++idx)
    {
        if (Utils::is_in(tdi[idx], 0, 0xFF)) // possible CLK
        {
            ++stable_tdi;
            if (stable_tdi >=3 ) // Terminate DATA run
            {
                ret_length = last_data+1;
                return BtST_Data;
            }
        }
        else
        {
            stable_tdi = 0;
            last_data = idx;
        }
    }
    return BtST_Data;
}

//////////////////////

class FTDI_JTAG : public Interf::JTAG {
    LibUsb usb;
    union {
        struct {
            uint8_t low_byte=FTDI_PIN_TMS, high_byte=0;
        };
        uint16_t both_bytes;
    };
    union {
        struct {
            uint8_t low_byte_dir=FTDI_PIN_TMS | FTDI_PIN_TDI | FTDI_PIN_TCK, high_byte_dir=0;
        };
        uint16_t both_bytes_dir;
    };
    std::unique_ptr<uint8_t[]> out_buffer;
    size_t out_buffer_ptr=0;

    void ftdi_init();
    void run_pin_seq(const char* name);
    void run_pin_seq(const Json::Value& value);
    void set_pins(int tms=-1, int tdi=-1);
    void upd_pin_image(uint16_t pin, bool value) {if (value) both_bytes |= pin; else both_bytes &= ~pin;}
    void upd_pin_dir(uint16_t pin, bool is_out) {if (value) both_bytes_dir |= pin; else both_bytes_dir &= ~pin;}

    uint8_t run_tms_tdi_bits(const BitSpanData&, uint8_t who, bool do_msb);
    uint8_t run_tms_bits(const BitSpanData& span) {return run_tms_tdi_bits(span, FTDI_MPSSE_BIT_WRITE_TMS, false);}
    uint8_t run_tdi_bits(const BitSpanData& span, bool do_msb) {return run_tms_tdi_bits(span, 0, do_msb);}

    void run_data_span(const ByteSpanData&, bool do_msb);
    void run_clk_span(const ByteSpanData&);

    void send_low_high_byte(uint8_t cmd, uint8_t value, uint8_t dir);
    void send_low_byte() {send_low_high_byte(FTDI_SET_LOW_BYTE, low_byte, low_byte_dir);}
    void send_high_byte() {send_low_high_byte(FTDI_SET_HIGH_BYTE, high_byte, high_byte_dir);}

    void putb(uint8_t byte) {if (out_buffer_ptr >= usb.in_endp_size()) flush(); out_buffer[out_buffer_ptr++] = byte;}
    void putd(const uint8_t* data, size_t size);
    void putw(uint16_t data) {putd(&data, 2);}
    void flush() {if (out_buffer_ptr) usb.write(out_buffer.get(), out_buffer_ptr); out_buffer_ptr=0;}

    void delay_as_string(const std::string& str);
    uint16_t collect_pin_list(const Json::Value& value, const char* key);

public:
    FTDI_JTAG() 
    {
        out_buffer.reset(new uint8_t[usb.in_endp_size()]);
        ftdi_init(); 
        run_pin_seq(".FTDI.start");
    }
    ~FTDI_JTAG() {run_pin_seq(".FTDI.close");}

    virtual void start() override {run_pin_seq(".FTDI.init");}
    virtual void end() override {run_pin_seq(".FTDI.fini");}

    virtual void shift(int bits, const uint8_t* tms, const uint8_t* tdi, uint8_t* tdo=NULL) override;
    virtual void shift_tdi(int bits, bool tms, const uint8_t* tdi, uint8_t* tdo=NULL, bool do_msb=false) override;
    virtual uint32_t set_frequency(uint32_t hz) override;
    virtual void clock(int total, bool tms_val, bool tdi_val) override
    {
        run_clk_span(ByteSpanData{.h_tms = tms, .h_tdi=tdi, .count=size_t(total)});
    }
    virtual int preallocate_tdo_bytes() override {return 2;} // 2 bytres for FTDI status bytes
};

void FTDI_JTAG::putd(const uint8_t* data, size_t size)
{
    while(size)
    {
        size_t free_place = usb.in_endp_size() - out_buffer_ptr;
        if (!free_place) {flush(); continue;}
        size_t wr = std::min(size, free_place);
        memcpy(out_buffer.get() + out_buffer_ptr, data, wr);
        out_buffer_ptr += wr;
        data += wr;
    }
}

void FTDI_JTAG::ftdi_init()
{
    static unsigned char startup[] = {
        FTDI_DISABLE_LOOPBACK,
        FTDI_DISABLE_3_PHASE_CLOCK,
        FTDI_SET_LOW_BYTE, FTDI_PIN_TMS, FTDI_PIN_TMS | FTDI_PIN_TDI | FTDI_PIN_TCK // Set low byte: <value>, <direction>
    };
    usb.control(BREQ_RESET, WVAL_RESET_RESET);
    usb.control(BREQ_SET_BITMODE, WVAL_SET_BITMODE_MPSSE);
    usb.control(BREQ_SET_LATENCY, 2);
    usb.control(BREQ_RESET, WVAL_RESET_PURGE_TX);
    usb.control(BREQ_RESET, WVAL_RESET_PURGE_RX)'
    set_frequency(10000000);
    usb.write(startup, sizeof(startup));
}

void FTDI_JTAG::shift(int bits, const uint8_t* tms, const uint8_t* tdi, uint8_t* tdo)
{
    SpanMgr span(bits, tms, tdi, tdo);
    while(auto span_type = span.has_span())
    {
        if (span_type == ST_Bit)
        {
            BitSpanData data;
            while(auto span_sybtype = span.get_bit_span(data))
            {
                uint8_t result = span_subtype == BiST_TMS ? run_tms_bits(data) : run_tdi_bits(data);
                span.put_bit_span_data(result);
            }
        }
        else // Byte span
        {
            ByteSpanData data;
            while(auto span_sybtype = span.get_byte_span(data))
            {
                if (span_sybtype == BtST_Data) run_data_span(data);
                else run_clk_span(data);
            }
        }
    }
    flush();
}

void FTDI_JTAG::shift_tdi(int bits, bool tms, const uint8_t* tdi, uint8_t* tdo, bool do_msb)
{
    int bytes = bits / 8;
    uint8_t* org_tdo;
    if (bytes)
    {
        ByteSpanData data {.h_tms = tms, .count = size_t(bytes), .tdi=tdi, .tdo=tdo};
        run_data_span(data, do_msb);
        tdi += bytes;
        if (tdo) tdo += bytes;
    }
    int rest = bits & 7;
    if (rest)
    {
        BitSpanData data {.static_val = tms, .need_tdo = tdo != NULL, *tms & ((1<<rest)-1)};
        uint8_t result = run_tdi_bits(data, do_msb);
        if (tdo) *tdo = result;
    }
    flush();
}

uint8_t FTDI_JTAG::run_tms_tdi_bits(const BitSpanData& span, uint8_t do_tms, bool do_msb)
{
    uint8_t result = 0;
    uint8_t cmd = FTDI_MPSSE_XFER_COMMON | do_tms | FTDI_MPSSE_BIT_BIT_MODE;
    if (do_msb) cmd ^= FTDI_MPSSE_BIT_LSB_FIRST;
    if (span.need_tdo) cmd |= FTDI_MPSSE_BIT_READ_DATA;
    if (do_tms) upd_pin_image(FTDI_PIN_TDI, span.shift >> 7); // TMS shift mode: TDI will be set by FTDI and encoded in bit 7 of span.shift
    else set_pins(span.static_val); // TDI shift mode. TMS not handled by FTDI, so manual update of TMS is needed
    putb(cmd);
    putb(span.len-1);
    putb(span.shift);
    if (span.need_tdo)
    {
        flush();
        usb.read(&result, 1);
    }
    bool last_bit_img = (do_msb ? span.shift : span.shift >> (span.len-1)) & 1;
    upd_pin_image(do_tms ? FTDI_PIN_TMS : FTDI_PIN_TDI, last_bit); // Update TMS/TDI image state
    return result;
}

void FTDI_JTAG::run_data_span(const ByteSpanData& span, bool do_msb)
{
    set_pins(span.h_tms);
    uint8_t cmd = FTDI_MPSSE_XFER_COMMON;
    if (span.tdo) cmd |= FTDI_MPSSE_BIT_READ_DATA;
    if (do_msb) cmd ^= FTDI_MPSSE_BIT_LSB_FIRST;
    putb(cmd);
    putw(span.count - 1);
    putd(span.tdi, span.count);
    if (span.tdo)
    {
        flush();
        usb.read(span.tdo, span.count);
    }
    uint8_t last_b = span.tdi[span.count - 1];
    if (!do_msb) last_b >>= 7;
    upd_pin_image(FTDI_PIN_TDI, last_b & 1); // Update TDI image state
}

void FTDI_JTAG::run_clk_span(const ByteSpanData& span)
{
    set_pins(span.h_tms, span.h_tdi);
    if (!span.count) return;
    if (span.count & 7)
    {
        putb(FTDI_MPSSE_CLOCK_BITS);
        putb((span.count & 7) + 1);
    }
    uint16_t cnt = (span.count / 8);
    if (cnt)
    {
        putb(FTDI_MPSSE_CLOCK_BYTES);
        putw(cnt-1);
    }
}

uint32_t FTDI_JTAG::set_frequency(uint32_t frequency)
{
    if (!frequency) frequency = 1;
    uint32_t divisor = std::min<size_t>(0x10000, std::max<size_t>(1, ((FTDI_CLOCK_RATE / 2) + (frequency - 1)) / frequency));
    uint32_t actual = FTDI_CLOCK_RATE / (2 * divisor);

    putb(FTDI_DISABLE_TCK_PRESCALER);
    putb(FTDI_SET_TCK_DIVISOR);
    putw(divisor-1);
    flush();

    return actual;
}

void FTDI_JTAG::send_low_high_byte(uint8_t cmd, uint8_t value, uint8_t dir)
{
    putb(cmd);
    putb(value);
    putb(dir);
}

void FTDI_JTAG::set_pins(int tms, int tdi)
{
    uint8_t l = low_byte;
    if (tms >= 0 ) upd_pin_image(FTDI_PIN_TMS, tms);
    if (tdi >= 0 ) upd_pin_image(FTDI_PIN_TDI, tdi);
    if (l != low_byte) send_low_byte();
}

void FTDI_JTAG::run_pin_seq(const char* name)
{
    auto value = Config::get(name);
    if (!value) return;
    if (value.isObject()) {run_pin_seq(value); return;}
    if (!value.isArray()) {error(std::format("Pin sequence '{}' has invalid type: extected Array of Object", name)); return;}
    for(const auto val: value) run_pin_seq(val);
}

void FTDI_JTAG::delay_as_string(const std::string& str)
{
    const char* c = str.c_str();
    char* e;
    auto dly_time = strtoul(c, &e, 10);
    if (!e) dly_time *= 1000000; else
    switch(tolower(*e))
    {
        case 'm': dly_time *= 1000000; break;
        case 'u': dly_time *= 1000; break;
        case 'n': break;
        default: error(std::format("Invalid delay time '{}'. Expected <num>[m|u|n]", str)); return;
    }
    Utils::delay_ns(dly_time);
}

uint16_t FTDI_JTAG::collect_pin_list(const Json::Value& value, const char* key)
{
    uint16_t result = 0;

    auto add_pin = [&, this](int val)
    {
        if (val <= 4 || val > 16) {error(std::format("Invalid Pin index '{}'. Valid range 4-16", val)); return;}
        int mask = 1 << (val - 1);
        if (result & mask) warning(std::format("Pin '{}' duplicated in list", val));
        result |= mask; 
    };

    if (!value.isMember(key)) return 0;

    auto plist = value[key];

    if (plist.isIntegral()) add_pin(plist.asUInt()); else
    if (!plist.isArray()) error("Pin list expected to be Array or Integer"); else
    for(const v: plist)
    {
        if (v.isIntegral()) add_pin(v.asUInt());
        else error("Pin expected to be Integer");
    }
    return result;
}


void FTDI_JTAG::run_pin_seq(const Json::Value& value)
{
    if (value.isIntegral())  // Delay in ms
    {
        Utils::delay_ms(value.asUInt());
        return;
    }
    if (value.isString())
    {
        delay_by_string(value.asString());
        return;
    }
    if (!value.isObject())
    {
        error("Pin sequence item has wrong format: Expected Object or String or Number");
        return;
    }
    uint16_t pin_high = collect_pin_list(value, "high");
    uint16_t pin_low  = collect_pin_list(value, "low");
    uint16_t pin_input= collect_pin_list(value, "input");

    if (pin_high & pin_low || pin_high & pin_input || pin_low & pin_input) 
    {
        error("Pin list in pin set Object intersects each other");
        return;
    }
    uint16_t p = pin_low | pin_high | pin_dir;
    if (p)
    {
        upd_pin_image(pin_low, false);
        upd_pin_image(pin_high, true);
        upd_pin_dir(pin_input, false);
        upd_pin_dir(pin_low|pin_high, true);
        if (p & 0xFF) send_low_byte();
        if (p & 0xFF00) send_high_byte();
        flush();
    }
    if (value.isMember("delay")) delay_by_string(value["delay"].asString());
}

}

Interf::JTAG* Interf::JTAG::create_ftdi_jtag() {return new FTDI_JTAG;}
