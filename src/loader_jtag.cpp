#include "common.h"
#include "interf.h"

namespace {

// TDI + TMS + Clocks
struct J {
    uint32_t tdi;
    uint32_t tms;
    uint8_t clocks;

    constexpr J(uint32_t tdi, int tms, uint8_t clocks) : tdi(tdi), tms(tms ? (1<<clocks)-1 : 0), clocks(clocks) {}

    constexpr J operator+ (const J other) const
    {
        return J(tdi | (other.tdi << clocks), tms | (other.tms << clocks), clocks + other.clocks);
    }
};

class JTAGLoader : public Loader {
    Interf::JTAG* jtag;

    void shift(const J &j) {assert(j.clocks < 32); jtag->shift(j.clocks, &j.tms, &j.tdi);}
public:
    JTAGLoader(Interf::JTAG* jtag) : jtag(jtag) {}

    virtual void start() override;
    virtual void load(const uint8_t* buffer, size_t size, bool is_last) override;
    virtual void end() override;
};


#define X 0

static uint64_t mks()
{
    timeval tv;
    gettimeofday(&tv);
    return tv.tv_sec * 1000000ull + tv.usec;
}

void JTAGLoader::start() override
{
    J start =                                               // TDI, TMS, Clocks
/* 1. On power-up, place a logic 1 on the TMS, and clock the 
      TCK five times. This ensures starting in the TLR  
       (Test-Logic-Reset) state.                        */     J(0,  1,  5) +
/*  2. Move into the RTI state. */                             J(X,  0,  1) +
/*  3. Move into the SELECT-IR state. */                       J(X,  1,  2) +
/*  4. Enter the SHIFT-IR state. */                            J(X,  0,  2) +
/*  5. Start loading the JPROGRAM instruction, LSB first:*/    J(11, 0,  5) + 
/*  6. Load the MSB of the JPROGRAM instruction when exiting                   
       SHIFT-IR, as defined in the IEEE standard. */           J(0,  1,  1)+
/*  7. Place a logic 1 on the TMS and clock the TCK five times.                
       This ensures starting in the TLR (Test- Logic-Reset) state. */ J(X,  1,  5);
    shift(start);
    //8. Wait here at least 10ms (we will wait 15) clocking 0/0 to JTAG
    auto start_time = mks();
    jtag->clock(1000, false, false);
    auto delta = mks()-start;
    if (delta < 15000) jtag->clock((15000-delta)*1000/delta, false, false);

    J cont =
/*9. Move into the SELECT-IR state.*/                    J(X, 1, 2)+
/*10. Enter the SHIFT-IR state.*/                        J(X, 0, 2)+
/*11. Start loading the CFG_IN instruction, LSB first:*/ J(5, 0, 5)+
/*12. Load the MSB of CFG_IN instruction when exiting
SHIFT-IR, as defined in the IEEE standard.*/             J(0, 1, 1)+
/*13. Enter the SELECT-DR state.*/                       J(X, 1, 2)+
/*14. Enter the SHIFT-DR state.*/                        J(X, 0, 2);
    shift(cont);
}

void JTAGLoader::load(const uint8_t* buffer, size_t size, bool is_last) override
{
// 15. Shift in the FPGA bitstream. Bitn (MSB) is the first bit in the bitstream
    jtag->shift_tdi(size*8 - last, false, buffer, true);
    if (last)
    {
        uint8_t l = buffer[size-1] >> 7;
        jtag->shift_tdi(1, true, &l);
    }
}

void JTAGLoader::end() override
{
    J last = 
/*17. Enter UPDATE-DR state.*/      J(X, 1, 1) +
/*18. Move into RTI state. */       J(X, 0, 1) +
/*19. Enter the SELECT-IR state.*/  J(X, 1, 2) +
/*20. Move to the SHIFT-IR state.*/ J(X, 0, 2) +
/*21. Start loading the JSTART instruction (optional). The
JSTART instruction initializes the startup sequence.*/ J(12, 0, 5) +
/*22. Load the last bit of the JSTART instruction.*/ J(0, 1, 1) +
/*23. Move to the UPDATE-IR state.*/ J(X, 1, 1);
    shift(last);
/*24. Move to the RTI state and clock the startup sequence by
applying a minimum of 2000 clock cycles to the TCK.*/
    jtag->clock(2000, false, false);
// Move to the TLR state. The device is now functional. 
    shift(J(X, 1, 3));
}


}


Interf::Loader* Interf::Loader::create_jtag_loader(Intef::JTAG* jtag) {return new JTAGLoader(jtag);}
                                                           