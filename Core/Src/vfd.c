#include "vfd.h"
#include "modbus.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

/* -----------------------------------------------------------------------
 * Integer formatting helpers — avoids float on Cortex-M3
 * fmt_d1(2350) → "235.0"   (÷10,  1 decimal place)
 * fmt_d2(2000) → "20.00"   (÷100, 2 decimal places)
 * ----------------------------------------------------------------------- */
static void fmt_d1(char *dst, uint16_t raw)
{
    /* raw = value * 10; result has 1 decimal place */
    sprintf(dst, "%u.%u", raw / 10, raw % 10);
}

static void fmt_d2(char *dst, uint16_t raw)
{
    /* raw = value * 100; result has 2 decimal places */
    sprintf(dst, "%u.%02u", raw / 100, raw % 100);
}

static void uart_print(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)strlen(msg), 1000);
}

/* -----------------------------------------------------------------------
 * VFD_ReadMonitoring
 *
 * Transaction 1: FC03, regs 1–9   (output_freq … motor_temp)
 * Transaction 2: FC03, regs 37–45 (fault_code, status_word, current_fixed)
 *
 * Returns 0 on success, non-zero if any Modbus transaction failed.
 * ----------------------------------------------------------------------- */
uint8_t VFD_ReadMonitoring(uint8_t slave_addr, VFD_Data *out)
{
    uint16_t     regs[9];
    ModbusStatus st;

    /* --- Block 1: registers 1–9 --- */
    st = Modbus_ReadHoldingRegs(slave_addr, VFD_REG_OUTPUT_FREQ, 9, regs);
    if (st != MODBUS_OK) return (uint8_t)st;

    out->output_freq     = regs[0];  /* reg 1  */
    out->motor_speed     = regs[1];  /* reg 2  */
    /* reg 3 (regs[2]) has frame-dependent scale — skipped; use reg 45 */
    out->motor_torque    = regs[3];  /* reg 4  */
    out->motor_power_pct = regs[4];  /* reg 5  */
    out->motor_voltage   = regs[5];  /* reg 6  */
    out->dc_link_voltage = regs[6];  /* reg 7  */
    out->unit_temp       = regs[7];  /* reg 8  */
    out->motor_temp      = regs[8];  /* reg 9  */

    /* --- Block 2: registers 37–45 (9 registers) --- */
    st = Modbus_ReadHoldingRegs(slave_addr, VFD_REG_FAULT_CODE, 9, regs);
    if (st != MODBUS_OK) return (uint8_t)st;

    out->fault_code  = regs[0];  /* reg 37 (offset 0) */
    out->status_word = regs[6];  /* reg 43 (offset 6) */
    out->current_a   = regs[8];  /* reg 45 (offset 8) */

    return 0;
}

/* -----------------------------------------------------------------------
 * VFD_ReadSolar
 * ----------------------------------------------------------------------- */
uint8_t VFD_ReadSolar(uint8_t slave_addr, VFD_Solar *out)
{
    uint16_t     reg;
    ModbusStatus st;

    st = Modbus_ReadHoldingRegs(slave_addr, VFD_REG_VMP_REF, 1, &reg);
    if (st != MODBUS_OK) return (uint8_t)st;
    out->vmp_ref = reg;

    /* Regs 1937–1938 are consecutive: read as a 2-register block */
    uint16_t blk[2];
    st = Modbus_ReadHoldingRegs(slave_addr, VFD_REG_ENERGY_COUNTER, 2, blk);
    if (st != MODBUS_OK) return (uint8_t)st;
    out->energy_mwh     = blk[0];  /* reg 1937 */
    out->motor_power_kw = blk[1];  /* reg 1938 */

    st = Modbus_ReadHoldingRegs(slave_addr, VFD_REG_IRRADIATION, 1, &reg);
    if (st != MODBUS_OK) return (uint8_t)st;
    out->irradiation = reg;

    st = Modbus_ReadHoldingRegs(slave_addr, VFD_REG_ACTUAL_FLOW, 1, &reg);
    if (st != MODBUS_OK) return (uint8_t)st;
    out->actual_flow = reg;

    return 0;
}

/* -----------------------------------------------------------------------
 * VFD_PrintMonitoring  — formatted output to UART1
 * ----------------------------------------------------------------------- */
void VFD_PrintMonitoring(const VFD_Data *d)
{
    char buf[80];
    char val[16];

    /* Status line */
    snprintf(buf, sizeof(buf), "\r\n--- VFD Status ---\r\n");
    uart_print(buf);

    /* Fault / ready */
    if (d->fault_code != 0)
        snprintf(buf, sizeof(buf), "  FAULT: %u\r\n", d->fault_code);
    else if (d->status_word & VFD_STATUS_RUN)
        snprintf(buf, sizeof(buf), "  State : RUNNING\r\n");
    else if (d->status_word & VFD_STATUS_READY)
        snprintf(buf, sizeof(buf), "  State : Ready (stopped)\r\n");
    else
        snprintf(buf, sizeof(buf), "  State : Not ready  (SW=0x%04X)\r\n",
                 d->status_word);
    uart_print(buf);

    /* Electrical */
    fmt_d2(val, d->output_freq);
    snprintf(buf, sizeof(buf), "  Freq  : %s Hz\r\n", val);
    uart_print(buf);

    snprintf(buf, sizeof(buf), "  Speed : %u rpm\r\n", d->motor_speed);
    uart_print(buf);

    fmt_d1(val, d->current_a);
    snprintf(buf, sizeof(buf), "  Curr  : %s A\r\n", val);
    uart_print(buf);

    fmt_d1(val, d->motor_voltage);
    snprintf(buf, sizeof(buf), "  Volt  : %s V\r\n", val);
    uart_print(buf);

    snprintf(buf, sizeof(buf), "  DC bus: %u V\r\n", d->dc_link_voltage);
    uart_print(buf);

    /* Load */
    fmt_d1(val, d->motor_torque);
    snprintf(buf, sizeof(buf), "  Torque: %s %%\r\n", val);
    uart_print(buf);

    fmt_d1(val, d->motor_power_pct);
    snprintf(buf, sizeof(buf), "  Power : %s %%\r\n", val);
    uart_print(buf);

    /* Temperatures */
    fmt_d1(val, d->unit_temp);
    snprintf(buf, sizeof(buf), "  Temp  : %s C (drive)\r\n", val);
    uart_print(buf);

    fmt_d1(val, d->motor_temp);
    snprintf(buf, sizeof(buf), "  MTemp : %s %% (motor)\r\n", val);
    uart_print(buf);
}

/* -----------------------------------------------------------------------
 * VFD_PrintSolar
 * ----------------------------------------------------------------------- */
void VFD_PrintSolar(const VFD_Solar *s)
{
    char buf[80];

    uart_print("\r\n--- Solar ---\r\n");

    snprintf(buf, sizeof(buf), "  Vmp ref    : %u V\r\n",   s->vmp_ref);
    uart_print(buf);
    snprintf(buf, sizeof(buf), "  Irradiation: %u W/m2\r\n", s->irradiation);
    uart_print(buf);
    snprintf(buf, sizeof(buf), "  Motor pwr  : %u kW\r\n",  s->motor_power_kw);
    uart_print(buf);
    snprintf(buf, sizeof(buf), "  Energy     : %u MWh\r\n", s->energy_mwh);
    uart_print(buf);
    snprintf(buf, sizeof(buf), "  Flow       : %u l/min\r\n", s->actual_flow);
    uart_print(buf);
}
