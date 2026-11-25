#include "au1300_uart.h"

namespace cerfcore {

namespace {

/* Linux arch/mips/include/asm/mach-au1x00/au1000.h AU1300_UART3_PHYS_ADDR. */
class Au1300Uart3 : public Au1300Uart {
public:
    using Au1300Uart::Au1300Uart;

    uint32_t MmioBase() const override { return 0x10103000u; }

protected:
    const char* Name() const override { return "UART3"; }
};

}  /* namespace */

REGISTER_SERVICE(Au1300Uart3);

}
