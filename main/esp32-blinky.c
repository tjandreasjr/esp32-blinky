#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_spi_flash.h"
#include "sdkconfig.h"

#define UART_PERIPH_NUM		0
#define UART_IO_TXD			1
#define UART_IO_RXD			3
#define UART_BAUDRATE     	115200
#define UART_BUFFER_SIZE	256

//#define UART_ECHO

#define UART_TASK_STACK_SIZE	4096
#define UART_TASK_PRIORITY		10

#define UART_CMD_LEDON		'h'
#define UART_CMD_LEDOFF		'l'
#define UART_CMD_LEDBLINK	'b'

#define LED_IO	2

#define TASK_DELAY	20	// ms

#define FLASH_ADDRESS	0x3FF000 // first word of the last sector on the 4 MB flash chip

#define LED_BLINK_SLOWDOWN_FACTOR 7

uint8_t set_led_on_command(uint32_t command, uint8_t led_state) {
	static uint8_t led_blink_dealias_counter = 0;

    switch (command) {
    case UART_CMD_LEDON:
    	led_blink_dealias_counter = 0;
    	gpio_set_level(LED_IO, 1);
    	led_state = 1;
    	break;
    case UART_CMD_LEDOFF:
    	led_blink_dealias_counter = 0;
    	gpio_set_level(LED_IO, 0);
    	led_state = 0;
    	break;
    case UART_CMD_LEDBLINK:
    	led_blink_dealias_counter++;
    	if (led_blink_dealias_counter >= LED_BLINK_SLOWDOWN_FACTOR) {
        	led_blink_dealias_counter = 0;
			led_state = !led_state;
			gpio_set_level(LED_IO, led_state);
    	}
		break;
    default:
		// default to LED off if the command byte is invalid (e.g. uninitialized Flash).
		led_blink_dealias_counter = 0;
		gpio_set_level(LED_IO, 0);
		led_state = 0;
    }

    return led_state;
}

// return true if the byte received over UART is one of the expected command bytes.
bool uart_data_is_valid(uint8_t uart_data) {
	return (uart_data == UART_CMD_LEDON ||
			uart_data == UART_CMD_LEDOFF ||
			uart_data == UART_CMD_LEDBLINK);
}

static void uart_blinky(void *arg)
{
	uint32_t flash_data = 0, uart_received_data = 0;
	uint8_t led_state = 0;
	uint8_t uart_buffer[UART_BUFFER_SIZE];

    // Setup UART port on UART0.
    uart_config_t uart_config = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

	// Configure blue LED output on IO2.
	gpio_config_t led_pin = {
		.pin_bit_mask = (1 << LED_IO),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE
	};
    gpio_config(&led_pin);

    uart_driver_install(UART_PERIPH_NUM, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PERIPH_NUM, &uart_config);
    uart_set_pin(UART_PERIPH_NUM, UART_IO_TXD, UART_IO_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // read from Flash to get the LED's intended initial state.
    ESP_ERROR_CHECK(spi_flash_read(FLASH_ADDRESS, &flash_data, 1));
    led_state = set_led_on_command(flash_data, led_state);

    while (1) {
        // read data from the UART.
        int uart_num_bytes = uart_read_bytes(UART_PERIPH_NUM, uart_buffer, UART_BUFFER_SIZE, TASK_DELAY / portTICK_RATE_MS);

        if (uart_num_bytes > 0) {
			// in case multiple bytes have been received, only the most recent byte is of interest.
			uart_received_data = uart_buffer[uart_num_bytes - 1];

    		// write data to Flash
			if (uart_data_is_valid(uart_received_data) && uart_received_data != flash_data) {
				flash_data = uart_received_data;
				ESP_ERROR_CHECK(spi_flash_erase_sector(FLASH_ADDRESS / SPI_FLASH_SEC_SIZE));
				ESP_ERROR_CHECK(spi_flash_write(FLASH_ADDRESS, &flash_data, 1));
			}
#ifdef UART_ECHO
			uart_write_bytes(UART_PERIPH_NUM, uart_buffer, uart_num_bytes);
#endif
        }

        // update LED activity
        led_state = set_led_on_command(flash_data, led_state);
    }
}

void app_main(void)
{
    xTaskCreate(uart_blinky, "uart_blinky", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL);
}
