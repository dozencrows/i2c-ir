// Report data as I2C slave on address 0x70.

#include "LPC8xx.h"
#include "lpc_types.h"
#include "romapi_8xx.h"
#include "serial.h"

#include <string.h>

#define FIXED_CLOCK_RATE_HZ     10000000
#define FIXED_UART_BAUD_RATE    115200

uint32_t i2cBuffer[24];     // data area used by ROM-based I2C driver
I2C_HANDLE_T* ih;           // opaque handle used by ROM-based I2C driver
I2C_PARAM_T i2cParam;       // input parameters for pending I2C request
I2C_RESULT_T i2cResult;     // return values for pending I2C request

uint8_t i2cRecvBuf[2];      // receive buffer: address + register number
uint8_t i2cSendBuf[32];     // send buffer 

void timersInit() {
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<10);    // enable MRT clock
    LPC_SYSCON->PRESETCTRL &= ~(1<<7);       // reset MRT
    LPC_SYSCON->PRESETCTRL |=  (1<<7);
    
    LPC_MRT->Channel[2].CTRL = (0x01 << 1); //MRT2 one-shot mode
}

void delayMs(int milliseconds) {
    LPC_MRT->Channel[2].INTVAL = (((FIXED_CLOCK_RATE_HZ / 250L) * milliseconds) >> 2) - 286;

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

void initSCTSquareWave() {
    // Turn on the clock to the SCT
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 8);
    
    // Set up output pin - (ISP)
    LPC_SWM->PINASSIGN6 = 0x01FFFFFF;

    LPC_SCT->CONFIG = 0;        // 16 bit counter timers
    
    // Set up two match events for halfway through and at the end
    LPC_SCT->REGMODE_L      = 0;
    LPC_SCT->MATCH[0].L     = SW_MATCH_PERIOD;
    LPC_SCT->MATCHREL[0].L  = SW_MATCH_PERIOD;
    LPC_SCT->MATCH[1].L     = SW_MATCH_HALF_PERIOD;
    LPC_SCT->MATCHREL[1].L  = SW_MATCH_HALF_PERIOD;
    
    // Toggle output pin on events to generate square wave
    LPC_SCT->OUT[0].SET     = 0x01;
    LPC_SCT->OUT[0].CLR     = 0x02;
    
    // Set up event control to simply enable events 1 and 2,
    // and associate them with L counter, match registers 0 and 1.
    LPC_SCT->EVENT[0].CTRL  = 0x5000;
    LPC_SCT->EVENT[0].STATE = 0x01;
    LPC_SCT->EVENT[1].CTRL  = 0x5001;
    LPC_SCT->EVENT[1].STATE = 0x01;
    
    // Set up counter limit on event 1 to restart
    LPC_SCT->LIMIT_L        = 0x01;    
}

void sctSquareWaveOn() {
    LPC_SCT->CTRL_L &= ~(1<<2);
}

void sctSquareWaveOff() {
    LPC_SCT->CTRL_L |= (1<<2);  // Stop
    LPC_SCT->CTRL_L |= (1<<3);  // Clear counter
    LPC_SCT->OUTPUT = 0;        // Clear output
}

void i2cSetupRecv (), i2cSetupSend (int);

void i2cSetup () {
    for (int i = 0; i < 3000000; ++i) __ASM("");

    LPC_SWM->PINENABLE0 |= 3<<2;            // disable SWCLK and SWDIO
    LPC_SWM->PINASSIGN7 = 0x02FFFFFF;       // SDA on P2, pin 4
    LPC_SWM->PINASSIGN8 = 0xFFFFFF03;       // SCL on P3, pin 3
    LPC_SYSCON->SYSAHBCLKCTRL |= 1<<5;  // enable I2C clock

    ih = LPC_I2CD_API->i2c_setup(LPC_I2C_BASE, i2cBuffer);
    LPC_I2CD_API->i2c_set_slave_addr(ih, 0x70<<1, 0);

    NVIC_EnableIRQ(I2C_IRQn);
}

extern "C" void I2C0_IRQHandler () {
    LPC_I2CD_API->i2c_isr_handler(ih);
}

void i2cRecvDone (uint32_t err, uint32_t) {
    i2cSetupRecv();
    if (err == 0)
        i2cSetupSend(i2cRecvBuf[1]);
}

void i2cSendDone (uint32_t err, uint32_t) {
    i2cSetupRecv();
}

void i2cSetupRecv () {
    i2cParam.func_pt = i2cRecvDone;
    i2cParam.num_bytes_send = 0;
    i2cParam.num_bytes_rec = 2;
    i2cParam.buffer_ptr_rec = i2cRecvBuf;
    LPC_I2CD_API->i2c_slave_receive_intr(ih, &i2cParam, &i2cResult);
}

void i2cSetupSend (int regNum) {
    i2cParam.func_pt = i2cSendDone;
    i2cParam.num_bytes_rec = 0;
    switch (regNum) {
        case 0:
            i2cParam.num_bytes_send = 1;
            i2cSendBuf[0] = 0xff;
            break;
            
        case 1:
            i2cParam.num_bytes_send = 16;
            strcpy((char*)i2cSendBuf, "0123456789ABCDEF");
            break;

        case 2:
            i2cParam.num_bytes_send = 1;
            i2cSendBuf[0] = 0xfe;
            sctSquareWaveOn();
            break;

        case 3:
            i2cParam.num_bytes_send = 1;
            i2cSendBuf[0] = 0xfd;
            sctSquareWaveOff();
            break;
    }
    i2cParam.buffer_ptr_send = i2cSendBuf;
    LPC_I2CD_API->i2c_slave_transmit_intr(ih, &i2cParam, &i2cResult);
}

int main () {
    initMainClock();
    serial.init(LPC_USART0, FIXED_UART_BAUD_RATE);
    delayMs(100);
    puts("i2c-ir started");
    i2cSetup();
    i2cSetupRecv();
    initSCTSquareWave();
    puts("Waiting...");
    while (true) {
        __WFI();
        puts("Msg received");
    }
}

