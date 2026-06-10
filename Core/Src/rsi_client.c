#include "rsi_client.h"
#include "modbus_master.h"
#include "debug_print.h"
#include <stdio.h>
#include <string.h>

/* Register addresses (Parameter ID = Modbus address on Vacon NXP / Grundfos RSI) */
#define REG_OUTPUT_FREQ     1u
#define REG_FAULT_CODE     37u
#define REG_STATUS_WORD    43u
#define REG_CURRENT_FIXED  45u
#define REG_VMP_REF      1914u
#define REG_ENERGY       1937u  /* also covers 1938 in a 2-reg read */
#define REG_FLOW         1956u
#define REG_IRRADIATION  1982u

/* Status word bit positions (1-indexed in docs → 0-indexed in code) */
#define SW_READY   0x0002u   /* B1 */
#define SW_RUN     0x0004u   /* B2 */
#define SW_FAULT   0x0008u   /* B3 */

/* -----------------------------------------------------------------------
 * RSI_Init
 * ----------------------------------------------------------------------- */
void RSI_Init(void)
{
    ModbusMaster_Init();
}

/* -----------------------------------------------------------------------
 * RSI_Poll
 *
 * Request A: regs  1– 9 (count=9) → core monitoring
 * Request B: regs 37–45 (count=9) → fault, status word, current (reg 45)
 * Request C: reg  1914  (count=1) → Vmp reference
 * Request D: regs 1937–1938 (count=2) → energy, motor power kW
 * Request E: reg  1956  (count=1) → flow
 * Request F: reg  1982  (count=1) → irradiation
 * ----------------------------------------------------------------------- */
int RSI_Poll(RSI_Data_t *data)
{
    uint16_t regs[9];
    int err;

    memset(data, 0, sizeof(*data));

    /* --- Request A: core monitoring --- */
    err = ModbusMaster_ReadHoldingRegisters(MODBUS_SLAVE_ADDR,
                                            REG_OUTPUT_FREQ, 9, regs);
    if (err) { ModbusLog_Error(err, REG_OUTPUT_FREQ); return err; }

    data->output_freq_hz   = (float)regs[0] / 100.0f;  /* reg 1  */
    data->motor_speed_rpm  = (int)regs[1];               /* reg 2  */
    /* regs[2] = reg 3 (varies scale) — ignored; current read from reg 45 */
    data->motor_torque_pct = (float)regs[3] / 10.0f;   /* reg 4  */
    data->motor_power_pct  = (float)regs[4] / 10.0f;   /* reg 5  */
    data->motor_voltage_v  = (float)regs[5] / 10.0f;   /* reg 6  */
    data->dc_link_voltage_v = (int)regs[6];              /* reg 7  */
    data->heatsink_temp_c  = (float)regs[7] / 10.0f;   /* reg 8  */
    data->motor_temp_pct   = (float)regs[8] / 10.0f;   /* reg 9  */

    /* --- Request B: fault, status, current ---
     * start=37, count=9 → indices: [0]=reg37 [6]=reg43 [8]=reg45 */
    err = ModbusMaster_ReadHoldingRegisters(MODBUS_SLAVE_ADDR,
                                            REG_FAULT_CODE, 9, regs);
    if (err) { ModbusLog_Error(err, REG_FAULT_CODE); return err; }

    data->fault_code       = (int)regs[0];    /* reg 37, offset 0 */
    data->drive_status_word = regs[6];         /* reg 43, offset 6 */
    data->motor_current_a  = (float)regs[8] / 10.0f;  /* reg 45, offset 8 */

    data->is_running = (data->drive_status_word & SW_RUN)   ? 1u : 0u;
    data->has_fault  = (data->drive_status_word & SW_FAULT) ? 1u : 0u;

    /* --- Request C: Vmp reference --- */
    err = ModbusMaster_ReadHoldingRegisters(MODBUS_SLAVE_ADDR,
                                            REG_VMP_REF, 1, regs);
    if (err) { ModbusLog_Error(err, REG_VMP_REF); return err; }
    data->vmp_ref_v = (int)regs[0];

    /* --- Request D: energy (1937) + motor power kW (1938) --- */
    err = ModbusMaster_ReadHoldingRegisters(MODBUS_SLAVE_ADDR,
                                            REG_ENERGY, 2, regs);
    if (err) { ModbusLog_Error(err, REG_ENERGY); return err; }
    data->energy_mwh    = (int)regs[0];
    data->motor_power_kw = (float)regs[1];   /* raw = kW, no divisor */

    /* --- Request E: flow --- */
    err = ModbusMaster_ReadHoldingRegisters(MODBUS_SLAVE_ADDR,
                                            REG_FLOW, 1, regs);
    if (err) { ModbusLog_Error(err, REG_FLOW); return err; }
    data->flow_lmin = (int)regs[0];

    /* --- Request F: irradiation --- */
    err = ModbusMaster_ReadHoldingRegisters(MODBUS_SLAVE_ADDR,
                                            REG_IRRADIATION, 1, regs);
    if (err) { ModbusLog_Error(err, REG_IRRADIATION); return err; }
    data->irradiation_wm2 = (int)regs[0];

    return 0;
}

/* -----------------------------------------------------------------------
 * RSI_BuildPayload  — compact JSON for forwarding (MQTT, UART, etc.)
 * ----------------------------------------------------------------------- */
void RSI_BuildPayload(const RSI_Data_t *d, char *buf, uint16_t buf_len)
{
    snprintf(buf, buf_len,
             "{\"freq\":%.2f,\"rpm\":%d,\"I\":%.1f,\"V\":%.1f,"
             "\"Vdc\":%d,\"T\":%.1f,"
             "\"flow\":%d,\"solar\":%d,\"kW\":%.1f,"
             "\"run\":%d,\"fault\":%d}",
             d->output_freq_hz, d->motor_speed_rpm,
             d->motor_current_a, d->motor_voltage_v,
             d->dc_link_voltage_v, d->heatsink_temp_c,
             d->flow_lmin, d->irradiation_wm2, d->motor_power_kw,
             (int)d->is_running, d->fault_code);
}

/* -----------------------------------------------------------------------
 * RSI_PrintDebug  — formatted table to USART1
 * ----------------------------------------------------------------------- */
void RSI_PrintDebug(const RSI_Data_t *d)
{
    printf("+-- Grundfos RSI -------------------------------------+\r\n");
    printf("| Output freq     %8.2f Hz                        |\r\n", d->output_freq_hz);
    printf("| Motor speed     %8d rpm                       |\r\n", d->motor_speed_rpm);
    printf("| Motor current   %8.1f A                         |\r\n", d->motor_current_a);
    printf("| Motor torque    %8.1f %%                         |\r\n", d->motor_torque_pct);
    printf("| Motor power     %8.1f %%  / %5.1f kW            |\r\n", d->motor_power_pct,
                                                                        d->motor_power_kw);
    printf("| Motor voltage   %8.1f V                         |\r\n", d->motor_voltage_v);
    printf("| DC link         %8d V                         |\r\n", d->dc_link_voltage_v);
    printf("| Heatsink temp   %8.1f C                         |\r\n", d->heatsink_temp_c);
    printf("| Motor temp      %8.1f %%                         |\r\n", d->motor_temp_pct);
    printf("| Status word     0x%04X  Run=%-3s Fault=%-3s        |\r\n",
           d->drive_status_word,
           d->is_running ? "YES" : "NO",
           d->has_fault  ? "YES" : "NO");
    printf("| Vmp ref         %8d V                         |\r\n", d->vmp_ref_v);
    printf("| Energy          %8d MWh                        |\r\n", d->energy_mwh);
    printf("| Flow            %8d l/min                      |\r\n", d->flow_lmin);
    printf("| Irradiation     %8d W/m2                       |\r\n", d->irradiation_wm2);
    if (d->has_fault)
        printf("| *** FAULT CODE: %-3d ***                             |\r\n", d->fault_code);
    printf("+-----------------------------------------------------+\r\n");
}
