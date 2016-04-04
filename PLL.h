#include <stdint.h>
#define PSYSDIV 5


// bus frequency is 480MHz/(PSYSDIV+1) = 480MHz/(3+1) = 120 MHz
// IMPORTANT: See Step 6) of PLL_Init().  If you change something, change 480 MHz.
// see the table at the end of this file

#define SYSCTL_RIS_R            (*((volatile uint32_t *)0x400FE050))
#define SYSCTL_RIS_MOSCPUPRIS   0x00000100  // MOSC Power Up Raw Interrupt
                                            // Status
#define SYSCTL_MOSCCTL_R        (*((volatile uint32_t *)0x400FE07C))
#define SYSCTL_MOSCCTL_PWRDN    0x00000008  // Power Down
#define SYSCTL_MOSCCTL_NOXTAL   0x00000004  // No Crystal Connected
#define SYSCTL_RSCLKCFG_R       (*((volatile uint32_t *)0x400FE0B0))
#define SYSCTL_RSCLKCFG_MEMTIMU 0x80000000  // Memory Timing Register Update
#define SYSCTL_RSCLKCFG_NEWFREQ 0x40000000  // New PLLFREQ Accept
#define SYSCTL_RSCLKCFG_USEPLL  0x10000000  // Use PLL
#define SYSCTL_RSCLKCFG_PLLSRC_M                                              \
                                0x0F000000  // PLL Source
#define SYSCTL_RSCLKCFG_PLLSRC_MOSC                                           \
                                0x03000000  // MOSC is the PLL input clock
                                            // source
#define SYSCTL_RSCLKCFG_OSCSRC_M                                              \
                                0x00F00000  // Oscillator Source
#define SYSCTL_RSCLKCFG_OSCSRC_MOSC                                           \
                                0x00300000  // MOSC is oscillator source
#define SYSCTL_RSCLKCFG_PSYSDIV_M                                             \
                                0x000003FF  // PLL System Clock Divisor
#define SYSCTL_MEMTIM0_R        (*((volatile uint32_t *)0x400FE0C0))
#define SYSCTL_DSCLKCFG_R       (*((volatile uint32_t *)0x400FE144))
#define SYSCTL_DSCLKCFG_DSOSCSRC_M                                            \
                                0x00F00000  // Deep Sleep Oscillator Source
#define SYSCTL_DSCLKCFG_DSOSCSRC_MOSC                                         \
                                0x00300000  // MOSC
#define SYSCTL_PLLFREQ0_R       (*((volatile uint32_t *)0x400FE160))
#define SYSCTL_PLLFREQ0_PLLPWR  0x00800000  // PLL Power
#define SYSCTL_PLLFREQ0_MFRAC_M 0x000FFC00  // PLL M Fractional Value
#define SYSCTL_PLLFREQ0_MINT_M  0x000003FF  // PLL M Integer Value
#define SYSCTL_PLLFREQ0_MFRAC_S 10
#define SYSCTL_PLLFREQ0_MINT_S  0
#define SYSCTL_PLLFREQ1_R       (*((volatile uint32_t *)0x400FE164))
#define SYSCTL_PLLFREQ1_Q_M     0x00001F00  // PLL Q Value
#define SYSCTL_PLLFREQ1_N_M     0x0000001F  // PLL N Value
#define SYSCTL_PLLFREQ1_Q_S     8
#define SYSCTL_PLLFREQ1_N_S     0
#define SYSCTL_PLLSTAT_R        (*((volatile uint32_t *)0x400FE168))
#define SYSCTL_PLLSTAT_LOCK     0x00000001  // PLL Lock
