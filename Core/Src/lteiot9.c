#include "lteiot9.h"
#include "defines.h"
#include <string.h>

void lteiot9_init(lteiot9_t *ctx, UART_HandleTypeDef * huart) {

    ctx->smi_port = LTEIOT9_SMI_GPIO_Port;
    ctx->smi_pin = LTEIOT9_SMI_Pin;

    ctx->on_port = LTEIOT9_ON_GPIO_Port;
    ctx->on_pin = LTEIOT9_ON_Pin;

    ctx->rts_port = LTEIOT9_RTS_GPIO_Port;
    ctx->rts_pin = GPIO_PIN_12;

    // Input pins
    ctx->cts_port = GPIOA;
    ctx->cts_pin = GPIO_PIN_11;

    // Modules
    ctx->uart = huart;

}

void lteiot9_set_on_pin(lteiot9_t *ctx, uint8_t state) {
    if (state > 0) {
        HAL_GPIO_WritePin(ctx->on_port, ctx->on_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(ctx->on_port, ctx->on_pin, GPIO_PIN_RESET);
    }
}

uint32_t lteiot9_generic_read ( lteiot9_t *ctx, char *data_buf, uint16_t max_len ) {

	HAL_UART_Receive(ctx->uart, (uint8_t*)data_buf, max_len, 1000);

	return sizeof(data_buf);

}

void lteiot9_send_cmd(lteiot9_t *ctx, char *cmd) {
    uint8_t cr_lf[] = {13, 10}; // Carriage return and line feed

    HAL_GPIO_WritePin(ctx->rts_port, ctx->rts_pin, GPIO_PIN_RESET); // Set RTS pin low

    // Wait until CTS pin is low
    while (HAL_GPIO_ReadPin(ctx->cts_port, ctx->cts_pin) == GPIO_PIN_SET);

    HAL_Delay(10); // Delay 10 ms

    // Send command
    while (*cmd != 0) {
        HAL_UART_Transmit(ctx->uart, (uint8_t *)cmd, 1, HAL_MAX_DELAY);
        cmd++;
    }

    // Send carriage return and line feed
    HAL_UART_Transmit(ctx->uart, cr_lf, 2, HAL_MAX_DELAY);

    HAL_Delay(100); // Delay 100 ms
}


void lteiot9_send_cmd_with_parameter ( lteiot9_t *ctx, char *at_cmd_buf, char *param_buf ) {
    char final_cmd[ 100 ] = { 0 };
    char check_char[ 2 ] = { '=', 0 };

    strcpy( final_cmd, at_cmd_buf );
    strcat( final_cmd, check_char );
    strcat( final_cmd, param_buf );

    lteiot9_send_cmd( ctx, final_cmd );
}

void lteiot9_send_cmd_check ( lteiot9_t *ctx, char *at_cmd_buf ) {
    char final_cmd[ 100 ] = { 0 };
    char check_char[ 2 ] = { '?', 0 };

    strcpy( final_cmd, at_cmd_buf );
    strcat( final_cmd, check_char );

    lteiot9_send_cmd( ctx, final_cmd );
}


void lteiot9_send_cmd_parameter_check ( lteiot9_t *ctx, char *at_cmd_buf ) {
    char final_cmd[ 100 ] = { 0 };
    char check_char[ 3 ] = { '=' , '?', 0 };

    strcpy( final_cmd, at_cmd_buf );
    strcat( final_cmd, check_char );

    lteiot9_send_cmd( ctx, final_cmd );
}

void lteiot9_set_sim_apn ( lteiot9_t *ctx, char *sim_apn ) {
    char final_cmd[ 50 ] = "1,\"IP\",\"";
    char end_cmd[ 3 ] = "\"";

    strcat( final_cmd, sim_apn );
    strcat( final_cmd, end_cmd );

    lteiot9_send_cmd_with_parameter( ctx, LTEIOT9_CMD_CGDCONT, final_cmd );
}

void lteiot9_send_text_message ( lteiot9_t *ctx, char *phone_number, char *message_context ) {
    char text[ 200 ] = { 0 };
    char cmd_start[] = "AT+CMGS=\"";
    char cmd_end[] = "\"\r\032";

    char txt_end[] = "\032";

    strcpy( text, cmd_start );
    strcat( text, phone_number );
    strcat( text, cmd_end );

    lteiot9_send_cmd( ctx, text );
    memset( text, 0 , 200 );

    strcpy( text, message_context );
    strcat( text, txt_end );
    lteiot9_send_cmd( ctx, text );
}
