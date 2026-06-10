#ifndef DEBUG_PRINT_H
#define DEBUG_PRINT_H

#include "main.h"
#include <stdio.h>

/* Timestamped log line — wraps printf, retargeted to USART1 via __io_putchar.
 * Requires -u _printf_float in linker flags for float format specifiers:
 *   CubeIDE → Project Properties → C/C++ Build → Settings →
 *   MCU GCC Linker → Miscellaneous → Other flags → add: -u _printf_float  */
#define LOG(fmt, ...) \
    printf("[%6lu] " fmt "\r\n", HAL_GetTick(), ##__VA_ARGS__)

/* Print a Modbus error line, e.g.: "[  1234] MODBUS ERR -2 (CRC) reg=1" */
void ModbusLog_Error(int err_code, uint16_t reg);

#endif /* DEBUG_PRINT_H */
