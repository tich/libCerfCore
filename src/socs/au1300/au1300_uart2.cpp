#include "au1300_uart.h"

namespace cerfcore {

namespace {

/* Linux arch/mips/include/asm/mach-au1x00/au1000.h AU1300_UART2_PHYS_ADDR. */
class Au1300Uart2 : public Au1300Uart {
public:
    using Au1300Uart::Au1300Uart;

    uint32_t MmioBase() const override { return 0x10102000u; }

protected:
    const char* Name() const override { return "UART2"; }
};

}  /* namespace */

REGISTER_SERVICE(Au1300Uart2);

}
