// Report data as I2C slave on address 0x70.

#include "LPC8xx.h"
#include "lpc_types.h"
#include "romapi_8xx.h"
#include "serial.h"

#include <string.h>

#define FIXED_CLOCK_RATE_HZ     12000000
#define FIXED_UART_BAUD_RATE    115200

uint32_t i2cBuffer[24];     // data area used by ROM-based I2C driver
I2C_HANDLE_T* ih;           // opaque handle used by ROM-based I2C driver
I2C_PARAM_T i2cParam;       // input parameters for pending I2C request
I2C_RESULT_T i2cResult;     // return values for pending I2C request

uint8_t i2cRecvBuf[2];      // receive buffer: address + register number
uint8_t i2cSendBuf[32];     // send buffer 

void delayMs(int milliseconds) {
    LPC_MRT->Channel[2].INTVAL = (((FIXED_CLOCK_RATE_HZ / 250L) * milliseconds) >> 2) - 286;

    while (LPC_MRT->Channel[2].STAT & 0x02)
        ; //wait while running
}

void i2cSetupRecv (), i2cSetupSend (int); // forward

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

// called when I2C reception has been completed
void i2cRecvDone (uint32_t err, uint32_t) {
    i2cSetupRecv();
    if (err == 0)
        i2cSetupSend(i2cRecvBuf[1]);
}

// called when I2C transmission has been completed
void i2cSendDone (uint32_t err, uint32_t) {
    i2cSetupRecv();
}

// prepare to receive the register number
void i2cSetupRecv () {
    i2cParam.func_pt = i2cRecvDone;
    i2cParam.num_bytes_send = 0;
    i2cParam.num_bytes_rec = 2;
    i2cParam.buffer_ptr_rec = i2cRecvBuf;
    LPC_I2CD_API->i2c_slave_receive_intr(ih, &i2cParam, &i2cResult);
}

// prepare to transmit either the byte count or the actual data
void i2cSetupSend (int regNum) {
    i2cParam.func_pt = i2cSendDone;
    i2cParam.num_bytes_rec = 0;
    if (regNum == 0) {
        i2cParam.num_bytes_send = 1;
        i2cSendBuf[0] = 0xff;
    } else {
        i2cParam.num_bytes_send = 16;
        strcpy((char*)i2cSendBuf, "0123456789ABCDEF");
    }
    i2cParam.buffer_ptr_send = i2cSendBuf;
    LPC_I2CD_API->i2c_slave_transmit_intr(ih, &i2cParam, &i2cResult);
}

int main () {
    serial.init(LPC_USART0, FIXED_UART_BAUD_RATE);
    delayMs(100);
    puts("i2c-ir started");
    i2cSetup();
    i2cSetupRecv();

    puts("Waiting...");
    while (true) {
        __WFI();
        puts("Msg received");
    }
}

