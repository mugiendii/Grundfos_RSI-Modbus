/* debug_print.c — printf retarget over USART1 + Modbus error logger
 *
 * IMPORTANT — linker flag required for float format specifiers:
 *   CubeIDE → Project Properties → C/C++ Build → Settings →
 *   MCU GCC Linker → Miscellaneous → Other flags → add: -u _printf_float
 *
 * newlib-nano's _write (in syscalls.c) calls __io_putchar for each byte;
 * implementing it here routes all printf output to USART1.              */

#include "debug_print.h"

extern UART_HandleTypeDef huart1;

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
    return ch;
}

void ModbusLog_Error(int err_code, uint16_t reg)
{
    const char *desc;
    switch (err_code) {
        case -1: desc = "TIMEOUT";    break;
        case -2: desc = "CRC";        break;
        case -3: desc = "EXCEPTION";  break;
        default: desc = "UNKNOWN";    break;
    }
    LOG("MODBUS ERR %d (%s) reg=%u", err_code, desc, (unsigned)reg);
}
