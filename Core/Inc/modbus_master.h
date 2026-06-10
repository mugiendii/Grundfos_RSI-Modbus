#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

#include "main.h"

#define MODBUS_SLAVE_ADDR   1u   /* default Grundfos RSI slave address */

/* Initialise DE/RE pin to RX mode and enforce the inter-frame gap. */
void ModbusMaster_Init(void);

/* FC03 — Read Holding Registers.
 * Returns  0   on success  (out_regs filled with 'count' big-endian uint16_t)
 *         -1   timeout or framing error
 *         -2   CRC mismatch
 *         -3   slave returned an exception response                         */
int ModbusMaster_ReadHoldingRegisters(uint8_t slave, uint16_t start_reg,
                                      uint16_t count, uint16_t *out_regs);

/* FC06 — Write Single Register.
 * Same return codes as above; -1 also covers response-echo mismatch.       */
int ModbusMaster_WriteSingleRegister(uint8_t slave, uint16_t reg,
                                     uint16_t value);

#endif /* MODBUS_MASTER_H */
