/*
 * lteiot9.h
 */

#ifndef INC_LTEIOT9_H_
#define INC_LTEIOT9_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

// AT commands
#define LTEIOT9_CMD_AT          "AT"
#define LTEIOT9_CMD_ATI         "ATI"
#define LTEIOT9_CMD_CGMR        "AT+CGMR"
#define LTEIOT9_CMD_CFUN        "AT+CFUN"
#define LTEIOT9_CMD_CREG        "AT+CREG"
#define LTEIOT9_CMD_CEREG       "AT+CEREG"
#define LTEIOT9_CMD_CGDCONT     "AT+CGDCONT"
#define LTEIOT9_CMD_CIMI        "AT+CIMI"
#define LTEIOT9_CMD_CGATT       "AT+CGATT"
#define LTEIOT9_CMD_CSQ         "AT+CSQ"
#define LTEIOT9_CMD_CESQ        "AT+CESQ"
#define LTEIOT9_CMD_COPS        "AT+COPS"
#define LTEIOT9_CMD_URAT        "AT+URAT"
#define LTEIOT9_CMD_UBANDMASK   "AT+UBANDMASK"
#define LTEIOT9_CMD_URATCONF    "AT+URATCONF"
#define LTEIOT9_CMD_UAUTHREQ    "AT+UAUTHREQ"
#define LTEIOT9_CMD_UUICC       "AT+UUICC"
#define LTEIOT9_CMD_UCGED       "AT+UCGED"
#define LTEIOT9_CMD_UCELLINFO   "AT+UCELLINFO"
#define LTEIOT9_CMD_UANTR       "AT+UANTR"
#define LTEIOT9_CMD_CMGF        "AT+CMGF"

// LTE IoT 9 GNNS helping
#define LTEIOT9_GNSS_GPGGA          "GPGGA"
#define LTEIOT9_GNSS_START          '$'
#define LTEIOT9_GNSS_SEPARATOR      ','

// LTE IoT 9 GPGGA value elements.
#define LTEIOT9_GPGGA_LATITUDE      2
#define LTEIOT9_GPGGA_LONGITUDE     4
#define LTEIOT9_GPGGA_ALTITUDE      9

// LTE IoT 9 start response.
#define LTEIOT9_SYSSTART            "^SYSSTART"

// LTE IoT 9 device response for AT commands.
#define LTEIOT9_RSP_OK                              "OK"
#define LTEIOT9_RSP_ERROR                           "ERROR"

// LTE IoT 9 driver buffer size.
#define DRV_TX_BUFFER_SIZE          200
#define DRV_RX_BUFFER_SIZE          500

#define ERR_OK           0   // No error
#define ERR_NOT_READY   -1   // UART or other component not ready
#define ERR_UART		-2

typedef enum
{
    LTEIOT9_OK = 0,
    LTEIOT9_ERROR = -1,
    LTEIOT9_ERROR_OVERFLOW = -2,
    LTEIOT9_ERROR_TIMEOUT = -3

} lteiot9_return_value_t;

typedef struct {
    // Output pins
    GPIO_TypeDef* smi_port;
    uint16_t smi_pin;

    GPIO_TypeDef* on_port;
    uint16_t on_pin;

    GPIO_TypeDef* rts_port;
    uint16_t rts_pin;

    // Input pins
    GPIO_TypeDef* cts_port;
    uint16_t cts_pin;

    // Modules
    UART_HandleTypeDef* uart;

} lteiot9_t;


void lteiot9_init(lteiot9_t *ctx, UART_HandleTypeDef * huart);

void lteiot9_set_on_pin ( lteiot9_t *ctx, uint8_t state );

uint32_t lteiot9_generic_read ( lteiot9_t *ctx, char *data_buf, uint16_t max_len );

void lteiot9_send_cmd(lteiot9_t *ctx, char *cmd);

void lteiot9_send_cmd_with_parameter ( lteiot9_t *ctx, char *at_cmd_buf, char *param_buf );

void lteiot9_send_cmd_check ( lteiot9_t *ctx, char *at_cmd_buf );

void lteiot9_send_cmd_parameter_check ( lteiot9_t *ctx, char *at_cmd_buf );

void lteiot9_set_sim_apn ( lteiot9_t *ctx, char *sim_apn );

void lteiot9_send_text_message ( lteiot9_t *ctx, char *phone_number, char *message_context );


#endif /* INC_LTEIOT9_H_ */
