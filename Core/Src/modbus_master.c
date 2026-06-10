#include "modbus_master.h"
#include "debug_print.h"
#include <string.h>

extern UART_HandleTypeDef huart3;

/* Timing — all in milliseconds.
 * At 9600 baud: 1 char = 10 bits = 1.042 ms
 *   Inter-frame gap (3.5 chars) = 3.65 ms → use 4 ms
 *   Inter-byte gap: must be > 1.5 chars (1.56 ms) and < 3.5 chars (3.65 ms);
 *   use 5 ms — slightly above the minimum inter-frame gap so we never cut a
 *   legitimate frame short even with HAL overhead.
 *   HAL_UART_Transmit timeout: 50 ms (8-byte frame = 8.3 ms @ 9600)        */
#define RESP_TIMEOUT_MS   1000
#define INTERBYTE_MS         5
#define INTERFRAME_MS        4
#define TX_TIMEOUT_MS       50

/* -----------------------------------------------------------------------
 * CRC16-Modbus  (poly 0xA001, init 0xFFFF)
 * ----------------------------------------------------------------------- */
static uint16_t crc16(uint8_t *buf, uint16_t len)
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
 * Clear UART error flags and reset HAL state.
 *
 * On STM32F1, ORE/FE/NE are cleared by reading SR then DR (not by writing
 * to SR).  When HAL_UART_Receive encounters these errors it sets
 * huart->ErrorCode and locks RxState to HAL_UART_STATE_ERROR; all future
 * calls return HAL_ERROR immediately until the state is reset here.
 * ----------------------------------------------------------------------- */
static void uart_clear_errors(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    __HAL_UART_CLEAR_FEFLAG(&huart3);
    __HAL_UART_CLEAR_NEFLAG(&huart3);
    huart3.ErrorCode = HAL_UART_ERROR_NONE;
    huart3.RxState   = HAL_UART_STATE_READY;
}

/* -----------------------------------------------------------------------
 * RS-485 transmit
 *
 * TC flag is cleared BEFORE HAL_UART_Transmit so the subsequent poll
 * reflects this frame only — not a stale flag from a previous transfer.
 * After HAL_UART_Transmit returns (TXE+TC waited by HAL), we still poll
 * TC explicitly as a belt-and-suspenders guarantee before dropping DE/RE.
 * 1 ms settle after DE/RE=LOW allows the MAX485 receiver to fully enable
 * before we start listening.
 * ----------------------------------------------------------------------- */
static void modbus_tx(const uint8_t *buf, uint16_t len)
{
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET);
    LOG("DBG tx: DE=1 bytes=%u [%02x %02x %02x %02x %02x %02x %02x %02x]",
        len, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
        len > 6 ? buf[6] : 0, len > 7 ? buf[7] : 0);
    __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_TC);
    HAL_StatusTypeDef tx_st = HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, TX_TIMEOUT_MS);
    LOG("DBG tx: HAL_st=%d gState=0x%02lx", (int)tx_st, huart3.gState);
    while (!__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC));
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
    HAL_Delay(1); /* MAX485 receiver enable settle */
}

/* -----------------------------------------------------------------------
 * RS-485 receive — byte-by-byte until inter-frame silence or buffer full.
 * Returns total bytes received (0 = first-byte timeout or UART error).
 * ----------------------------------------------------------------------- */
static int modbus_rx(uint8_t *buf, uint16_t max_len)
{
    uint16_t count = 0;
    uart_clear_errors();

    HAL_StatusTypeDef rx_st = HAL_UART_Receive(&huart3, &buf[0], 1, RESP_TIMEOUT_MS);
    LOG("DBG rx: st=%d ec=0x%02lx SR=0x%03x",
        (int)rx_st, huart3.ErrorCode,
        (unsigned)(huart3.Instance->SR & 0x3FF));
    if (rx_st != HAL_OK) return 0;
    count = 1;
    while (count < max_len &&
           HAL_UART_Receive(&huart3, &buf[count], 1, INTERBYTE_MS) == HAL_OK)
        count++;

    return (int)count;
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */
void ModbusMaster_Init(void)
{
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
    uart_clear_errors(); /* drain any noise captured during boot */
    HAL_Delay(INTERFRAME_MS);
}

/* FC03 — Read Holding Registers */
int ModbusMaster_ReadHoldingRegisters(uint8_t slave, uint16_t start_reg,
                                      uint16_t count, uint16_t *out_regs)
{
    uint8_t  req[8], resp[128];
    uint16_t crc, rx_crc;
    int      rx_len;

    req[0] = slave;
    req[1] = 0x03;
    req[2] = (uint8_t)(start_reg >> 8);
    req[3] = (uint8_t)(start_reg & 0xFF);
    req[4] = (uint8_t)(count >> 8);
    req[5] = (uint8_t)(count & 0xFF);
    crc    = crc16(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);
    req[7] = (uint8_t)(crc >> 8);

    HAL_Delay(INTERFRAME_MS);
    modbus_tx(req, 8);

    rx_len = modbus_rx(resp, (uint16_t)sizeof(resp));
    if (rx_len == 0)                               return -1;
    if (rx_len < 5)                                return -1;

    rx_crc = (uint16_t)resp[rx_len - 1] << 8 | resp[rx_len - 2];
    if (crc16(resp, (uint16_t)(rx_len - 2)) != rx_crc) return -2;

    if (resp[1] & 0x80)                            return -3;
    if (resp[0] != slave || resp[1] != 0x03)       return -1;
    if (resp[2] != (uint8_t)(count * 2))           return -1;
    if (rx_len  != (int)(5 + count * 2))           return -1;

    for (uint16_t i = 0; i < count; i++)
        out_regs[i] = (uint16_t)resp[3 + i * 2] << 8 | resp[4 + i * 2];

    return 0;
}

/* FC06 — Write Single Register */
int ModbusMaster_WriteSingleRegister(uint8_t slave, uint16_t reg,
                                     uint16_t value)
{
    uint8_t  req[8], resp[8];
    uint16_t crc, rx_crc;
    int      rx_len;

    req[0] = slave;
    req[1] = 0x06;
    req[2] = (uint8_t)(reg   >> 8);
    req[3] = (uint8_t)(reg   & 0xFF);
    req[4] = (uint8_t)(value >> 8);
    req[5] = (uint8_t)(value & 0xFF);
    crc    = crc16(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);
    req[7] = (uint8_t)(crc >> 8);

    HAL_Delay(INTERFRAME_MS);
    modbus_tx(req, 8);

    rx_len = modbus_rx(resp, (uint16_t)sizeof(resp));
    if (rx_len < 8)              return -1;

    rx_crc = (uint16_t)resp[7] << 8 | resp[6];
    if (crc16(resp, 6) != rx_crc)    return -2;

    if (resp[1] & 0x80)          return -3;
    if (resp[0] != slave)        return -1;
    if (resp[1] != 0x06)         return -1;
    if (memcmp(req, resp, 6) != 0)   return -1;

    return 0;
}
