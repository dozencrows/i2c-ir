//=======================================================================
// Copyright Nicholas Tuckett 2015.
// Distributed under the MIT License.
// (See accompanying file license.txt or copy at
//  http://opensource.org/licenses/MIT)
//=======================================================================

//#define DEBUG

#include "LPC8xx.h"
#include "lpc_types.h"

#include <string.h>

#if defined(DEBUG)
#define FIXED_UART_BAUD_RATE    115200

#include "serial.h"

const char hexdigit[] = "0123456789abcdef";

void puthex8(uint8_t v) {
    putchar(hexdigit[(v >> 4) & 0xf]);
    putchar(hexdigit[v & 0xf]);
}

void puthex32(unsigned int v) {
    for (int i = 24; i >= 0; i-=4) {
        putchar(hexdigit[(v >> i) & 0xf]);
    }
}
#endif

#define FIXED_CLOCK_RATE_HZ     10000000
#define DELAY_MRT               2
#define SLEEP_MRT               1
#define IR_STAGE_COUNT  64
#define DEFER_SLEEP_TIME_MS     4000

typedef union IrRegisters {
    struct {
        uint8_t     status;
        uint8_t     repeats;
        uint8_t     repeat_delay;
        uint8_t     length;
        uint16_t    timing[IR_STAGE_COUNT];
    } v;
    uint8_t         m[4 + IR_STAGE_COUNT * sizeof(uint16_t)];
} IrRegisters;

void timersInit() {
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<10);    // enable MRT clock
    LPC_SYSCON->PRESETCTRL &= ~(1<<7);       // reset MRT
    LPC_SYSCON->PRESETCTRL |=  (1<<7);
    
    LPC_MRT->Channel[SLEEP_MRT].CTRL = 0x03;        // Sleep timer in one-shot mode, interrupt generated
    LPC_MRT->Channel[DELAY_MRT].CTRL = 0x01 << 1;   // Delay timer in one-shot mode, no interrupt generated
    
    NVIC_EnableIRQ(MRT_IRQn);
}

void delayMs(int milliseconds) {
    LPC_MRT->Channel[DELAY_MRT].INTVAL = (FIXED_CLOCK_RATE_HZ / 1000L) * milliseconds;

    while (LPC_MRT->Channel[DELAY_MRT].STAT & 0x02)
        ; //wait while running
}

volatile uint8_t g_sleepFlag = 0;

void deferSleep(int milliseconds) {
    g_sleepFlag = 0;
    LPC_MRT->Channel[SLEEP_MRT].INTVAL = (FIXED_CLOCK_RATE_HZ / 1000L) * milliseconds | (1 << 31);;
}

extern "C" void MRT_IRQHandler(void) {
    g_sleepFlag = 1;
    LPC_MRT->Channel[SLEEP_MRT].STAT = 0x1;
}

void initMainClock() {
    LPC_SYSCON->PDRUNCFG    &= ~(0x1 << 7);     // Power up PLL
    LPC_SYSCON->SYSPLLCTRL  = 0x24;             // MSEL=4, PSEL=1 -> M=5, P=2 -> Fclkout = 60Mhz
    while (!(LPC_SYSCON->SYSPLLSTAT & 0x01));   // Wait for PLL lock

    LPC_SYSCON->MAINCLKSEL    = 0x03;           // Set main clock to PLL source and wait for update
    LPC_SYSCON->SYSAHBCLKDIV  = 6;              // Set divider to get final system clock of 10Mhz
    LPC_SYSCON->MAINCLKUEN    = 0x00;
    LPC_SYSCON->MAINCLKUEN    = 0x01;
    while (!(LPC_SYSCON->MAINCLKUEN & 0x01));
    
    LPC_SYSCON->PDAWAKECFG   &= ~(1 << 7);      // Ensure PLL powers up after wakeup
    
    LPC_FLASHCTRL->FLASHCFG     = 0;            // Switch to zero wait-states for Flash
}

#define SW_FREQ                 38000
#define SW_MATCH_PERIOD         (FIXED_CLOCK_RATE_HZ / SW_FREQ)
#define SW_MATCH_HALF_PERIOD    (SW_MATCH_PERIOD / 2)

// Times are in tenths of uS
volatile uint8_t g_nextCycle    = 0;
volatile uint8_t g_waveDone     = 0;
volatile uint8_t g_repeatCount  = 0;
volatile IrRegisters g_irRegisters;

extern "C" void SCT_IRQHandler(void) {
    uint32_t events = LPC_SCT->EVFLAG;
    
    if (events & 0x08) {
        if (g_nextCycle < g_irRegisters.v.length) {
            LPC_SCT->MATCHREL[2].H  = g_irRegisters.v.timing[g_nextCycle];
            LPC_SCT->MATCHREL[0].H  = g_irRegisters.v.timing[g_nextCycle++] + g_irRegisters.v.timing[g_nextCycle++];
        }
        else {
            // To finish, event 0 set to halt both timers, clear output and fire final event
            LPC_SCT->HALT_L     = 0x01;
            LPC_SCT->HALT_H     = 0x01;
            LPC_SCT->OUT[0].CLR = 0x01;
            LPC_SCT->EVEN       = 0x01;
        }
    }
    else if (events & 0x01) {
        g_waveDone = 1;
    }
        
    LPC_SCT->EVFLAG = events;
}

void initIR() {
    memset((void*)&g_irRegisters, 0, sizeof(g_irRegisters));
    
    // ---------------------------------
    // -- Common timer initialisation --
    
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 8);  // Turn on the clock to the SCT
    LPC_SYSCON->PRESETCTRL &= ~(1<<8);      // reset SCT
    LPC_SYSCON->PRESETCTRL |=  (1<<8);
    LPC_SWM->PINASSIGN6 = 0x00FFFFFF;       // Set up output pin - (RXD/PIO0_0)
    LPC_SCT->CONFIG = (1<<17)|(1<<18);      // H and L 16 bit counter timers, using match reg 0 each to limit

    // ------------------------------------------------------
    // -- L timer - generates carrier square wave at 38kHz --

    // Use match reg 0 to define end of cycle, and act as auto limit
    // Use match regs 1 and 2 to define events 0 and 1 for start and
    // middle of cycle, turning output on and off respectively
    LPC_SCT->MATCH[0].L     = SW_MATCH_PERIOD;
    LPC_SCT->MATCHREL[0].L  = SW_MATCH_PERIOD;
    LPC_SCT->MATCH[1].L     = 0;
    LPC_SCT->MATCHREL[1].L  = 0;
    LPC_SCT->MATCH[2].L     = SW_MATCH_HALF_PERIOD;
    LPC_SCT->MATCHREL[2].L  = SW_MATCH_HALF_PERIOD;

    LPC_SCT->EVENT[0].CTRL  = 0x5001;
    LPC_SCT->EVENT[0].STATE = 0x01;
    LPC_SCT->EVENT[1].CTRL  = 0x5002;
    LPC_SCT->EVENT[1].STATE = 0x01;
    
    LPC_SCT->OUT[0].SET     = 0x01;

    // ----------------------------------------
    // -- H timer - modulates L timer on/off --
    
    // Match reg 0 defines overall H timer cycle.
    // Event 2 turns on timer L, and occurs at start of H cycle (via match reg 1)
    LPC_SCT->MATCH[1].H     = 0;
    LPC_SCT->MATCHREL[1].H  = 0;
    LPC_SCT->EVENT[2].CTRL  = 0x5011;
    LPC_SCT->EVENT[2].STATE = 0x01;
    LPC_SCT->START_L        = 0x04;
    
    // Event 3 turns off timer L and output, and occurs in middle portion of H cycle
    // (via match reg 2) - and fires interrupt to configure the following H cycle.
    LPC_SCT->EVENT[3].CTRL  = 0x5012;
    LPC_SCT->EVENT[3].STATE = 0x01;
    LPC_SCT->STOP_L         = 0x08;

    NVIC_EnableIRQ(SCT_IRQn);
}

void irSendSignal(int repeats) {
    g_irRegisters.v.status = 1;
    
    LPC_SCT->HALT_L     = 0x00;
    LPC_SCT->HALT_H     = 0x00;
    LPC_SCT->OUT[0].CLR = 0x0a;     // events 1 and 3 clear output
     
    // Set L counter into stopped but unhalted
    uint16_t ctrl_l = LPC_SCT->CTRL_L;
    ctrl_l &= ~(1<<2);
    ctrl_l |= 1<<1;
    LPC_SCT->CTRL_L = ctrl_l;
    
    // Configure & start H counter (which will start L counter)
    g_nextCycle = 0;
    LPC_SCT->MATCH[2].H  = g_irRegisters.v.timing[g_nextCycle];
    LPC_SCT->MATCH[0].H  = g_irRegisters.v.timing[g_nextCycle++] + g_irRegisters.v.timing[g_nextCycle++];
    
    LPC_SCT->EVEN = 0x08;
    g_waveDone = 0;
    LPC_SCT->CTRL_L |= (1<<3);          // Clear L counter
    LPC_SCT->CTRL_H |= (1<<3)|(9<<5);   // Clear H counter, set prescale divide to factor of 10
                                        // so that H counter is in microseconds
    
    if (repeats > 0) {
        g_repeatCount = repeats;
    }
    
    LPC_SCT->CTRL_H &= ~(1<<2);   
}

void irStopSignal() {
    LPC_SCT->CTRL_U |= (1<<2)|(1<<18);  // Halt counters
    LPC_SCT->CTRL_U |= (1<<3)|(1<<19);  // Clear counters
    LPC_SCT->OUTPUT = 0;                // Clear output
    g_repeatCount = 0;
    g_waveDone = 0;
    g_irRegisters.v.status = 0;
}

void irUpdateSignal() {
    if (g_waveDone) {
        g_repeatCount--;
        if (g_repeatCount) {
            delayMs(g_irRegisters.v.repeat_delay);
            irSendSignal(0);
        }
        else {
#if defined(DEBUG)        
            puts("Sent");
#endif
            irStopSignal();
        }
    }
}

//
// Notes on I2C...
//
// i2cset -y 1 0x70 A V b           - sends 3 bytes: 0xe0 A V where A is address and V is value
// i2cset -y 1 0x70 A V1 V2 V3 i    - sends 5 bytes: 0xe0 A V1 V2 V3
//
// i2cdump =y -r N-M 1 0x70 c       - sends e0 N | (e1 rx |) x (M-N+1)
//

#define I2C_CFG_SLVEN           0x02
#define I2C_SLVPENDING          (1<<8)
#define I2C_SLVDESELCT          (1<<15)
#define I2C_STAT_SLVSTATE       (3<<9)
#define I2C_STAT_SLVST_ADDR     0x00
#define I2C_STAT_SLVST_RX       (1<<9)
#define I2C_STAT_SLVST_TX       (2<<9)
#define I2C_SLVCTL_SLVCONTINUE  0x01
#define I2C_SLVCTL_SLVNAK       0x02

void i2cSetup () {
    //for (int i = 0; i < 3000000; ++i) __ASM("");

    LPC_SWM->PINENABLE0 |= 3<<2;            // disable SWCLK and SWDIO
    LPC_SWM->PINASSIGN7 = 0x02FFFFFF;       // SDA on P2, pin 4
    LPC_SWM->PINASSIGN8 = 0xFFFFFF03;       // SCL on P3, pin 3
    LPC_IOCON->PIO0_2 &= ~(0x18);           // Disable pullups
    LPC_IOCON->PIO0_3 &= ~(0x18);           // Disable pullups

    LPC_SYSCON->SYSAHBCLKCTRL |= 1<<5;      // enable I2C clock
    LPC_SYSCON->PRESETCTRL &= ~(1<<6);      // reset I2C
    LPC_SYSCON->PRESETCTRL |=  (1<<6);
    
    // Configure I2C: address, slave mode, interrupt on slave pending or deselect
    LPC_I2C->SLVADR0    = 0x70 << 1;
    LPC_I2C->CFG        = I2C_CFG_SLVEN;
    LPC_I2C->INTENSET   = I2C_SLVPENDING | I2C_SLVDESELCT;

    NVIC_EnableIRQ(I2C_IRQn);
}

#define I2C_STATE_IDLE          0
#define I2C_STATE_RX_REGISTER   1
#define I2C_STATE_RX_DATA       2
#define I2C_STATE_TX_DATA       3

unsigned int g_currRegister = 0;
unsigned int g_i2cState = I2C_STATE_IDLE;

extern "C" void I2C0_IRQHandler () {
    uint32_t intstat    = LPC_I2C->INTSTAT;

    if (intstat & I2C_SLVPENDING) {
        uint32_t state  = LPC_I2C->STAT & I2C_STAT_SLVSTATE;
        uint32_t result = I2C_SLVCTL_SLVCONTINUE;
        uint8_t  data   = LPC_I2C->SLVDAT;

        switch (state) {
            case I2C_STAT_SLVST_ADDR:
                g_i2cState = I2C_STATE_RX_REGISTER;
                break;
                
            case I2C_STAT_SLVST_RX:
                switch (g_i2cState) {
                    case I2C_STATE_RX_REGISTER:
                        g_currRegister = data;
                        g_i2cState = I2C_STATE_RX_DATA;
                        break;

                    case I2C_STATE_RX_DATA:
                        if (g_irRegisters.v.status) {
                            result = I2C_SLVCTL_SLVNAK;
                        }
                        else if (g_currRegister > 0 && g_currRegister < sizeof(IrRegisters)) {
                            g_irRegisters.m[g_currRegister++] = data;
                        }
                        break;
                };
                break;
                
            case I2C_STAT_SLVST_TX:
                g_i2cState = I2C_STATE_TX_DATA;
                if (g_currRegister == 0) {
                    LPC_I2C->SLVDAT = g_irRegisters.v.status;
                }
                else {
                    LPC_I2C->SLVDAT = 0;
                    result = I2C_SLVCTL_SLVNAK;
                }
                break;
                
            default:
                g_i2cState = I2C_STATE_IDLE;
                break;
        }

        if (result) {        
            LPC_I2C->SLVCTL = result;
        }
    }
    
    if (intstat & I2C_SLVDESELCT) {
        if (g_i2cState == I2C_STATE_RX_DATA && g_currRegister > offsetof(IrRegisters, v.timing)) {

#if defined(DEBUG)        
            puthex8(g_irRegisters.v.repeats);
            putchar(' ');
            puthex8(g_irRegisters.v.repeat_delay);
            putchar(' ');
            puthex8(g_irRegisters.v.length);
            for (unsigned int i = 0; i < g_irRegisters.v.length; i++) {
                putchar(' ');
                puthex32(g_irRegisters.v.timing[i]);
            }
            putchar('\n');
#endif

            irSendSignal(g_irRegisters.v.repeats);
        }
        g_i2cState = I2C_STATE_IDLE;
        LPC_I2C->STAT |= I2C_SLVDESELCT;
        deferSleep(DEFER_SLEEP_TIME_MS);
    }
}

static void deepSleep() {
    LPC_SYSCON->STARTERP1   = (1 << 8); // I2C interrupt will wakeup 
    LPC_PMU->PCON           = 0x01;     // select deep sleep
    SCB->SCR                = SCB_SCR_SLEEPDEEP_Msk;
    __WFI();
    LPC_PMU->PCON           = 0;
    SCB->SCR                = 0;
}

static void powerDown() {
    LPC_SYSCON->STARTERP1   = (1 << 8);                 // I2C interrupt will wakeup 
    LPC_PMU->PCON           = 0x02;                     // select power down
    SCB->SCR                = SCB_SCR_SLEEPDEEP_Msk;
    __WFI();
    LPC_PMU->PCON           = 0;
    SCB->SCR                = 0;
}

static void configureLowPowerPins() {
    // Set PIO0_10 and 11 as low outputs, as they float.
    // This saves some power
    LPC_GPIO_PORT->DIR0 |= 3 << 10;
    LPC_GPIO_PORT->CLR0  = 3 << 10;
}

int main () {
    initMainClock();
    timersInit();
    configureLowPowerPins();

#if defined(DEBUG)        
    serial.init(LPC_USART0, FIXED_UART_BAUD_RATE);
    delayMs(100);
#endif
    
#if defined(DEBUG)        
    puts("i2c-ir started");
#endif

    i2cSetup();
    initIR();

#if defined(DEBUG)        
    puts("Waiting...");
#endif

    deferSleep(DEFER_SLEEP_TIME_MS);
    
    while (true) {
        if (g_sleepFlag && !g_irRegisters.v.status) {
            powerDown();
            deferSleep(DEFER_SLEEP_TIME_MS);
        }
        else {
            __WFI();
        }
        irUpdateSignal();
    }
}

