#ifndef RSI_CLIENT_H
#define RSI_CLIENT_H

#include "main.h"

/* -----------------------------------------------------------------------
 * Decoded drive data — all fields in engineering units after scaling
 * ----------------------------------------------------------------------- */
typedef struct {
    float    output_freq_hz;     /* reg 1,    ÷100  → Hz       */
    int      motor_speed_rpm;    /* reg 2,    ÷1    → rpm      */
    float    motor_current_a;    /* reg 45,   ÷10   → A  (fixed scale, all frames) */
    float    motor_torque_pct;   /* reg 4,    ÷10   → %        */
    float    motor_power_pct;    /* reg 5,    ÷10   → %        */
    float    motor_voltage_v;    /* reg 6,    ÷10   → V        */
    int      dc_link_voltage_v;  /* reg 7,    ÷1    → V        */
    float    heatsink_temp_c;    /* reg 8,    ÷10   → °C       */
    float    motor_temp_pct;     /* reg 9,    ÷10   → %        */
    uint16_t drive_status_word;  /* reg 43,   raw bits          */
    int      fault_code;         /* reg 37,   0 = no fault      */
    int      vmp_ref_v;          /* reg 1914, ÷1    → V        */
    int      energy_mwh;         /* reg 1937, ÷1    → MWh      */
    float    motor_power_kw;     /* reg 1938, ÷1    → kW       */
    int      flow_lmin;          /* reg 1956, ÷1    → l/min    */
    int      irradiation_wm2;    /* reg 1982, ÷1    → W/m²     */
    uint8_t  is_running;         /* derived: status_word & 0x0004 (bit 2) */
    uint8_t  has_fault;          /* derived: status_word & 0x0008 (bit 3) */
} RSI_Data_t;

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */

void RSI_Init(void);

/* Poll all registers in 6 FC03 requests.
 * Returns 0 on success, or the error code of the first failed request.    */
int  RSI_Poll(RSI_Data_t *data);

/* Build a compact JSON payload for forwarding (UART, MQTT, etc.).
 * Output is always null-terminated and never exceeds buf_len bytes.       */
void RSI_BuildPayload(const RSI_Data_t *data, char *buf, uint16_t buf_len);

/* Print a formatted table to USART1 (via printf / debug_print).           */
void RSI_PrintDebug(const RSI_Data_t *data);

#endif /* RSI_CLIENT_H */
