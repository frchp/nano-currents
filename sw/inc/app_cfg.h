#ifndef _APP_CFG_H_
#define _APP_CFG_H_

/**
 * Message size
 */
#define UART_MSG_SIZE         (5u)

/**
 * Send raw or computed data
 */
#define CFG_SEND_RAW_DATA

/**
 * Board info
 */
#define CURRENT_AMP_GAIN      (500u)
#define CURRENT_AMP_SHUNT     (10u)

#define VOLTAGE_AMP_GAIN      (1u)
#define VOLTAGE_AMP_IN_RATIO  (0.5f)
#define VOLTAGE_AMP_INV_RATIO (2u) // = 1/VOLTAGE_AMP_IN_RATIO

#endif // _APP_CFG_H_