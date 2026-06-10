#include "modbus.h"

/* huart3 is the RS-485 UART, declared as a global in main.c */
extern UART_HandleTypeDef huart3;

#define RESP_TIMEOUT_MS    1000  /* max wait for first byte of response  */
#define INTERBYTE_MS          5  /* max silence between bytes in a frame */
#define INTERFRAME_MS         4  /* min silence before sending a request
                                    (>= 3.5 char times @ 9600 = ~3.65 ms) */

/* -----------------------------------------------------------------------
 * CRC16-Modbus  (poly 0x8005 reflected = 0xA001, init 0xFFFF)
 * ----------------------------------------------------------------------- */
static uint16_t crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

/* -----------------------------------------------------------------------
 * RS-485 physical layer
 * ----------------------------------------------------------------------- */
static void bus_tx(const uint8_t *data, uint16_t len)
{
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET);
    __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_TC);
    HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 1000);
    while (!__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC));
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
    HAL_Delay(1); /* MAX485 receiver enable settle */
}

/* Returns number of bytes actually received.
 * First byte uses a long timeout (waiting for slave to respond);
 * subsequent bytes use a short inter-byte timeout to detect frame end. */
static uint16_t bus_rx(uint8_t *buf, uint16_t max_len, uint32_t first_timeout_ms)
{
    uint16_t count = 0;
    if (HAL_UART_Receive(&huart3, &buf[0], 1, first_timeout_ms) != HAL_OK)
        return 0;
    count = 1;
    while (count < max_len &&
           HAL_UART_Receive(&huart3, &buf[count], 1, INTERBYTE_MS) == HAL_OK)
        count++;
    return count;
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */
void Modbus_Init(void)
{
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET); /* RX mode */
    HAL_Delay(INTERFRAME_MS);
}

/* FC03 — Read Holding Registers ---------------------------------------- */
ModbusStatus Modbus_ReadHoldingRegs(uint8_t addr, uint16_t reg,
                                    uint16_t count, uint16_t *out)
{
    uint8_t  req[8];
    uint8_t  resp[128];
    uint16_t rx_len, crc;

    /* Build 8-byte request */
    req[0] = addr;
    req[1] = 0x03;
    req[2] = (uint8_t)(reg   >> 8);
    req[3] = (uint8_t)(reg   & 0xFF);
    req[4] = (uint8_t)(count >> 8);
    req[5] = (uint8_t)(count & 0xFF);
    crc    = crc16(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);   /* CRC low byte first */
    req[7] = (uint8_t)(crc >> 8);

    HAL_Delay(INTERFRAME_MS);
    bus_tx(req, 8);

    /* Expected response: [addr][0x03][byteCount][regHi][regLo]...[crcL][crcH]
     *                     1  +   1  +     1     + count*2        +  2  bytes */
    rx_len = bus_rx(resp, sizeof(resp), RESP_TIMEOUT_MS);
    if (rx_len == 0)                             return MODBUS_ERR_TIMEOUT;
    if (rx_len < 5)                              return MODBUS_ERR_FRAME;
    if (resp[0] != addr)                         return MODBUS_ERR_ADDRESS;
    if (resp[1] == (0x03 | 0x80))                return MODBUS_ERR_EXCEPTION;
    if (resp[1] != 0x03)                         return MODBUS_ERR_FUNCTION;
    if (resp[2] != (uint8_t)(count * 2))         return MODBUS_ERR_FRAME;
    if (rx_len  != (uint16_t)(5 + count * 2))    return MODBUS_ERR_FRAME;

    /* Verify CRC (covers everything except the last 2 bytes) */
    uint16_t rx_crc = (uint16_t)resp[rx_len - 1] << 8 | resp[rx_len - 2];
    if (crc16(resp, rx_len - 2) != rx_crc)       return MODBUS_ERR_CRC;

    /* Unpack register values (big-endian in frame) */
    for (uint16_t i = 0; i < count; i++)
        out[i] = (uint16_t)resp[3 + i * 2] << 8 | resp[4 + i * 2];

    return MODBUS_OK;
}

/* FC06 — Write Single Register ----------------------------------------- */
ModbusStatus Modbus_WriteSingleReg(uint8_t addr, uint16_t reg, uint16_t value)
{
    uint8_t  req[8], resp[8];
    uint16_t rx_len, crc;

    req[0] = addr;
    req[1] = 0x06;
    req[2] = (uint8_t)(reg   >> 8);
    req[3] = (uint8_t)(reg   & 0xFF);
    req[4] = (uint8_t)(value >> 8);
    req[5] = (uint8_t)(value & 0xFF);
    crc    = crc16(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);
    req[7] = (uint8_t)(crc >> 8);

    HAL_Delay(INTERFRAME_MS);
    bus_tx(req, 8);

    /* Slave echoes the request unchanged */
    rx_len = bus_rx(resp, sizeof(resp), RESP_TIMEOUT_MS);
    if (rx_len == 0)              return MODBUS_ERR_TIMEOUT;
    if (rx_len < 8)               return MODBUS_ERR_FRAME;
    if (resp[0] != addr)          return MODBUS_ERR_ADDRESS;
    if (resp[1] == (0x06 | 0x80)) return MODBUS_ERR_EXCEPTION;
    if (resp[1] != 0x06)          return MODBUS_ERR_FUNCTION;

    uint16_t rx_crc = (uint16_t)resp[7] << 8 | resp[6];
    if (crc16(resp, 6) != rx_crc) return MODBUS_ERR_CRC;

    return MODBUS_OK;
}

/* ----------------------------------------------------------------------- */
const char *Modbus_StatusStr(ModbusStatus s)
{
    switch (s) {
        case MODBUS_OK:             return "OK";
        case MODBUS_ERR_TIMEOUT:    return "TIMEOUT";
        case MODBUS_ERR_CRC:        return "CRC_ERR";
        case MODBUS_ERR_ADDRESS:    return "ADDR_ERR";
        case MODBUS_ERR_FUNCTION:   return "FC_ERR";
        case MODBUS_ERR_EXCEPTION:  return "EXCEPTION";
        case MODBUS_ERR_FRAME:      return "FRAME_ERR";
        default:                    return "UNKNOWN";
    }
}
