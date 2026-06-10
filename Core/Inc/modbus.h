#ifndef MODBUS_H
#define MODBUS_H

#include "main.h"

/* -----------------------------------------------------------------------
 * Modbus RTU master — FC03 Read Holding Registers, FC06 Write Single Reg
 * ----------------------------------------------------------------------- */

typedef enum {
    MODBUS_OK            = 0,
    MODBUS_ERR_TIMEOUT   = 1,  /* no response from slave */
    MODBUS_ERR_CRC       = 2,  /* response CRC mismatch */
    MODBUS_ERR_ADDRESS   = 3,  /* response slave address doesn't match */
    MODBUS_ERR_FUNCTION  = 4,  /* unexpected function code in response */
    MODBUS_ERR_EXCEPTION = 5,  /* slave returned an exception code */
    MODBUS_ERR_FRAME     = 6,  /* response length or byte-count wrong */
} ModbusStatus;

void         Modbus_Init(void);

/* Read 'count' holding registers starting at 'reg' from slave 'addr'.
 * Results written into out[0..count-1] as big-endian uint16_t. */
ModbusStatus Modbus_ReadHoldingRegs(uint8_t addr, uint16_t reg,
                                    uint16_t count, uint16_t *out);

/* Write a single 16-bit value to register 'reg' on slave 'addr'. */
ModbusStatus Modbus_WriteSingleReg(uint8_t addr, uint16_t reg, uint16_t value);

const char  *Modbus_StatusStr(ModbusStatus s);

#endif /* MODBUS_H */
