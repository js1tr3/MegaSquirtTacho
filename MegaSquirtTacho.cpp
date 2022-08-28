#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/divider.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/pwm.h"
#include "include/mcp2515/mcp2515.h"

MCP2515 can0;
struct can_frame rx;
// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 9600

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 4
#define UART_RX_PIN 5

// GPIO defines
// Example uses GPIO 2
#define GPIO 2


// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

#define Tacho_PIN 24
// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}

#define PWM_1k 1000
#define PWM_5k 65535

int main()
{
    __uint16_t RPM=0;

    stdio_init_all();

    // Set up our UART
    uart_init(UART_ID, BAUD_RATE);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    

    // Tell the tacho pin that the PWM is in charge of its value.
    gpio_set_function(Tacho_PIN, GPIO_FUNC_PWM);
    // Figure out which slice we just connected to the LED pin
    uint slice_num = pwm_gpio_to_slice_num(Tacho_PIN);

    // Get some sensible defaults for the slice configuration. By default, the
    // counter is allowed to wrap over its maximum range (0 to 2**16-1)
    pwm_config config = pwm_get_default_config();
    // Set divider, reduces counter clock to sysclock/this value
    pwm_config_set_clkdiv(&config, 1.f);
    // Load the configuration into our PWM slice, and set it running.
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(Tacho_PIN, 0);


    // GPIO initialisation.
    // We will make this GPIO an input, and pull it up by default
    gpio_init(GPIO);
    gpio_set_dir(GPIO, GPIO_IN);
    gpio_pull_up(GPIO);
    

    // Example of using the HW divider. The pico_divider library provides a more user friendly set of APIs 
    // over the divider (and support for 64 bit divides), and of course by default regular C language integer
    // divisions are redirected thru that library, meaning you can just use C level `/` and `%` operators and
    // gain the benefits of the fast hardware divider.
    int32_t dividend = 123456;
    int32_t divisor = -321;
    // This is the recommended signed fast divider for general use.
    divmod_result_t result = hw_divider_divmod_s32(dividend, divisor);
    printf("%d/%d = %d remainder %d\n", dividend, divisor, to_quotient_s32(result), to_remainder_s32(result));
    // This is the recommended unsigned fast divider for general use.
    int32_t udividend = 123456;
    int32_t udivisor = 321;
    divmod_result_t uresult = hw_divider_divmod_u32(udividend, udivisor);
    printf("%d/%d = %d remainder %d\n", udividend, udivisor, to_quotient_u32(uresult), to_remainder_u32(uresult));

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Timer example code - This example fires off the callback after 2000ms
    add_alarm_in_ms(2000, alarm_callback, NULL, false);


    puts("Hello, world!");
    stdio_init_all();

    //Initialize interface
    can0.reset();
    can0.setBitrate(CAN_500KBPS, MCP_16MHZ);
    can0.setNormalMode();

    //Listen loop
    while(true) {
        if(can0.readMessage(&rx) == MCP2515::ERROR_OK) {
            printf("New frame from ID: %10x\n", rx.can_id);

        }
        int16_t ch = getchar_timeout_us(100);
        while (ch != PICO_ERROR_TIMEOUT) {
           // uart_putc_raw(UART0_ID, ch);    // Send to UART0
           // uart_putc_raw(UART1_ID, ch);    // Send to UART1
           // printf("%c", ch);               // Echo back so you can see what you've done
            ch = getchar_timeout_us(100);
        }
        switch(ch){
        case 1:
            pwm_set_gpio_level(Tacho_PIN, 1000);
            printf("PWM: %d\n",1000);
        break;
                case 2:
            pwm_set_gpio_level(Tacho_PIN, 10000);
            printf("PWM: %d\n",10000);
        break;
                case 3:
            pwm_set_gpio_level(Tacho_PIN, 20000);
            printf("PWM: %d\n",20000);
        break;
                case 4:
            pwm_set_gpio_level(Tacho_PIN, 30000);
            printf("PWM: %d\n",30000);
        break;
                case 5:
            pwm_set_gpio_level(Tacho_PIN, 50000);
        break;
                case 0:
            pwm_set_gpio_level(Tacho_PIN, 0);
        break;
        }
    }

    return 0;
}


