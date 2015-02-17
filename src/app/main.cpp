// Report data as I2C slave on address 0x70.

#include "LPC8xx.h"
#include "lpc_types.h"
#include "romapi_8xx.h"
#include "serial.h"

#include <string.h>

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

#define FIXED_CLOCK_RATE_HZ     10000000
#define FIXED_UART_BAUD_RATE    115200

#define IR_STAGE_COUNT  64

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

uint32_t i2cBuffer[24];     // data area used by ROM-based I2C driver
I2C_HANDLE_T* ih;           // opaque handle used by ROM-based I2C driver
I2C_PARAM_T i2cParam;       // input parameters for pending I2C request
I2C_RESULT_T i2cResult;     // return values for pending I2C request

#define I2C_RECV_SIZE   (sizeof(IrRegisters) + 2)
uint8_t i2cRecvBuf[I2C_RECV_SIZE];  // receive buffer: address + register number + data
uint8_t i2cSendBuf[2];              // send buffer: address + status register

void timersInit() {
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<10);    // enable MRT clock
    LPC_SYSCON->PRESETCTRL &= ~(1<<7);       // reset MRT
    LPC_SYSCON->PRESETCTRL |=  (1<<7);
    
    LPC_MRT->Channel[2].CTRL = (0x01 << 1); //MRT2 one-shot mode
}

void delayMs(int milliseconds) {
    LPC_MRT->Channel[2].INTVAL = (FIXED_CLOCK_RATE_HZ / 1000L) * milliseconds;

    while (LPC_MRT->Channel[2].STAT & 0x02)
        ; //wait while running
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
}

#define SW_FREQ                 38000
#define SW_MATCH_PERIOD         (FIXED_CLOCK_RATE_HZ / SW_FREQ)
#define SW_MATCH_HALF_PERIOD    (SW_MATCH_PERIOD / 2)

// Times are in tenths of uS
volatile uint8_t g_nextCycle    = 0;
volatile uint8_t g_waveDone     = 0;
uint8_t g_repeatCount           = 0;
IrRegisters g_irRegisters;

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
    memset(&g_irRegisters, 0, sizeof(g_irRegisters));
    
    // ---------------------------------
    // -- Common timer initialisation --
    
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 8);  // Turn on the clock to the SCT
    LPC_SYSCON->PRESETCTRL &= ~(1<<8);      // reset SCT
    LPC_SYSCON->PRESETCTRL |=  (1<<8);
    LPC_SWM->PINASSIGN6 = 0x01FFFFFF;       // Set up output pin - (ISP)
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
}

void irUpdateSignal() {
    if (g_waveDone) {
        g_repeatCount--;
        if (g_repeatCount) {
            delayMs(g_irRegisters.v.repeat_delay);
            irSendSignal(0);
        }
        else {
            puts("Done.");
            g_waveDone = 0;
        }
    }
}

//
// Notes on I2C...
//
// i2cset -y 1 0x70 A V b           - sends 3 bytes: 0xe0 A V where A is address and V is value
// i2cset -y 1 0x70 A V1 V2 V3 i    - sends 5 bytes: 0xe0 A V1 V2 V3
//
// Looks like i2c_slave_receive_intr expects exactly the given number of bytes to be received to
// then call the callback... if sent less, it accumulates those in the buffer until a send happens
// with the right number expected (and corresponding stop bit).
//
// It would seem that a more custom I2C driver needs writing...
 
void i2cSetupRecv (), i2cSetupSend (uint8_t, uint8_t);

void i2cSetup () {
    for (int i = 0; i < 3000000; ++i) __ASM("");

    LPC_SWM->PINENABLE0 |= 3<<2;            // disable SWCLK and SWDIO
    LPC_SWM->PINASSIGN7 = 0x02FFFFFF;       // SDA on P2, pin 4
    LPC_SWM->PINASSIGN8 = 0xFFFFFF03;       // SCL on P3, pin 3
    LPC_SYSCON->SYSAHBCLKCTRL |= 1<<5;      // enable I2C clock

    ih = LPC_I2CD_API->i2c_setup(LPC_I2C_BASE, i2cBuffer);
    LPC_I2CD_API->i2c_set_slave_addr(ih, 0x70<<1, 0);

    NVIC_EnableIRQ(I2C_IRQn);
}

extern "C" void I2C0_IRQHandler () {
    LPC_I2CD_API->i2c_isr_handler(ih);
}

void i2cRecvDone (uint32_t err, uint32_t n) {
    puthex32(err);
    putchar(' ');
    puthex32(n);
    putchar('\n');
    
    i2cSetupRecv();
    if (err == 0) {
        for (int i = 0; i < n; i++) {
            puthex8(i2cRecvBuf[i]);
            putchar(' ');
        }
        putchar('\n');
    }
}

void i2cSendDone (uint32_t err, uint32_t) {
    i2cSetupRecv();
}

void i2cSetupRecv () {
    i2cParam.func_pt = i2cRecvDone;
    i2cParam.num_bytes_send = 0;
    i2cParam.num_bytes_rec = 8; //I2C_RECV_SIZE;
    i2cParam.buffer_ptr_rec = i2cRecvBuf;
    i2cParam.stop_flag = 1;
    LPC_I2CD_API->i2c_slave_receive_intr(ih, &i2cParam, &i2cResult);
}

void i2cSetupSend (uint8_t value) {
    i2cParam.func_pt = i2cSendDone;
    i2cParam.num_bytes_rec = 0;
    i2cParam.num_bytes_send = 1;
    i2cSendBuf[0] = value;
    i2cParam.buffer_ptr_send = i2cSendBuf;
    LPC_I2CD_API->i2c_slave_transmit_intr(ih, &i2cParam, &i2cResult);
}

int main () {
    initMainClock();
    timersInit();
    serial.init(LPC_USART0, FIXED_UART_BAUD_RATE);
    delayMs(100);
    puts("i2c-ir started");
    i2cSetup();
    i2cSetupRecv();
    initIR();
    puts("Waiting...");
    while (true) {
        __WFI();
        irUpdateSignal();
    }
}

