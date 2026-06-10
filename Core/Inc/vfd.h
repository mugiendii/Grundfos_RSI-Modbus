#ifndef VFD_H
#define VFD_H

#include "main.h"

/* -----------------------------------------------------------------------
 * VFD Modbus register map (Parameter ID = Modbus address)
 * ----------------------------------------------------------------------- */

/* --- Core monitoring (FC03, read as a 9-register block starting at 1) --- */
#define VFD_REG_OUTPUT_FREQ       1    /* Hz   raw÷100  e.g. 2000 = 20.00 Hz */
#define VFD_REG_MOTOR_SPEED       2    /* rpm  raw÷1                          */
#define VFD_REG_MOTOR_CURRENT     3    /* A    scale varies by frame size      */
#define VFD_REG_MOTOR_TORQUE      4    /* %    raw÷10                          */
#define VFD_REG_MOTOR_POWER_PCT   5    /* %    raw÷10                          */
#define VFD_REG_MOTOR_VOLTAGE     6    /* V    raw÷10                          */
#define VFD_REG_DC_LINK_VOLTAGE   7    /* V    raw÷1                           */
#define VFD_REG_UNIT_TEMP         8    /* °C   raw÷10                          */
#define VFD_REG_MOTOR_TEMP        9    /* %    raw÷10  % of nominal temp       */

/* --- Fault / status (read as a 9-register block starting at 37) --- */
#define VFD_REG_FAULT_CODE        37   /* 0 = no fault                         */
#define VFD_REG_STATUS_WORD       43   /* see VFD_STATUS_* bits below          */
#define VFD_REG_CURRENT_FIXED     45   /* A    raw÷10  fixed scale all frames  */

/* --- Solar-specific (individual reads) --- */
#define VFD_REG_VMP_REF           1914 /* V    MPP tracker DC voltage ref      */
#define VFD_REG_ENERGY_COUNTER    1937 /* MWh  total PV energy                 */
#define VFD_REG_MOTOR_POWER_KW    1938 /* kW   absolute motor shaft power      */
#define VFD_REG_VMP_CORRECTION    1942 /* V    P&O correction on DC ref        */
#define VFD_REG_ACTUAL_FLOW       1956 /* l/min from flow transducer           */
#define VFD_REG_IRRADIATION       1982 /* W/m² from irradiation sensor         */

/* --- Status word bit masks (reg 43) --- */
#define VFD_STATUS_READY          (1u << 1)   /* B1 — ready to run */
#define VFD_STATUS_RUN            (1u << 2)   /* B2 — running      */
#define VFD_STATUS_FAULT          (1u << 3)   /* B3 — fault active */

/* -----------------------------------------------------------------------
 * Data structures
 * ----------------------------------------------------------------------- */

typedef struct {
    /* All values stored raw — apply the stated divisor before displaying */
    uint16_t output_freq;      /* ÷100  → Hz    */
    uint16_t motor_speed;      /* ÷1    → rpm   */
    uint16_t motor_torque;     /* ÷10   → %     */
    uint16_t motor_power_pct;  /* ÷10   → %     */
    uint16_t motor_voltage;    /* ÷10   → V     */
    uint16_t dc_link_voltage;  /* ÷1    → V     */
    uint16_t unit_temp;        /* ÷10   → °C    */
    uint16_t motor_temp;       /* ÷10   → %     */
    uint16_t fault_code;       /* 0 = no fault  */
    uint16_t status_word;      /* VFD_STATUS_*  */
    uint16_t current_a;        /* ÷10   → A  (fixed-scale reg 45) */
} VFD_Data;

typedef struct {
    uint16_t vmp_ref;          /* V   — MPP tracker reference  */
    uint16_t energy_mwh;       /* MWh — lifetime energy        */
    uint16_t motor_power_kw;   /* kW  — absolute shaft power   */
    uint16_t irradiation;      /* W/m²                         */
    uint16_t actual_flow;      /* l/min                        */
} VFD_Solar;

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */

/* Read core monitoring data (3 Modbus transactions) */
uint8_t VFD_ReadMonitoring(uint8_t slave_addr, VFD_Data *out);

/* Read solar-specific registers (individual transactions) */
uint8_t VFD_ReadSolar(uint8_t slave_addr, VFD_Solar *out);

/* Print all fields to a UART handle — uses UART1 (debug port) */
void    VFD_PrintMonitoring(const VFD_Data *d);
void    VFD_PrintSolar(const VFD_Solar *s);

#endif /* VFD_H */
