#include <mach/platform.h> /* included for BCM2709 & BCM2708 definitions, e.g. BCM2708_PERI_BASE */


/* on newer kernel the following defines are defined in mach/platform.h 
 * on older kernels, the following defines are missing and we need to define them ourself */

#ifndef BCM2708_PERI_BASE

#define BCM2708_PERI_BASE 0x20000000
#define GPIO_BASE (BCM2708_PERI_BASE + 0x200000)
#define SPI0_BASE (BCM2708_PERI_BASE + 0x204000)

#endif

#define SPI0_CNTLSTAT *(Spi0 + 0)
#define SPI0_FIFO *(Spi0 + 1)
#define SPI0_CLKSPEED *(Spi0 + 2)

// SPI transfer done. WRT to CLR!
#define SPI0_CS_DONE     0x00010000 

/* spi transfer active */
#define SPI0_TA 1 << 7

// Activate: be high before starting
#define SPI0_CS_ACTIVATE 0x00000080
#define SPI0_CS_CLRFIFOS 0x00000030
#define SPI0_CS_CHIPSEL0 0x00000000
#define SPI0_CS_CHIPSEL1 0x00000001
#define SPI0_CS_CHIPSEL2 0x00000002
#define SPI0_CS_CHIPSELN 0x00000003

#define SPI0_CS_CLRALL (SPI0_CS_CLRFIFOS | SPI0_CS_DONE)
