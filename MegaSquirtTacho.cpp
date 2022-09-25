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
#include "hardware/clocks.h"
#include "megasquirt_simplified_dash_broadcast.h"
MCP2515 can0;
struct can_frame rx;
// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
//#define UART_ID uart1
//#define BAUD_RATE 9600

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
//#define UART_TX_PIN 4
//#define UART_RX_PIN 5

// GPIO defines
// Example uses GPIO 2
#define GPIO 20
#define GPIO_O 18


// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 4
#define PIN_CS   9
#define PIN_SCK  2
#define PIN_MOSI 13

#define Tacho_PIN 24
// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c1
#define I2C_SDA 6
#define I2C_SCL 7

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}

#define PWM_1k 1000
#define PWM_5k 65535

int main()
{
    __uint16_t RPM=0;
    megasquirt_simplified_dash_broadcast_megasquirt_dash0_t dash0_data;
    const uint32_t f_pwm = 3000; // frequency we want to generate
	uint16_t duty = 60; // duty cycle, in percent
    stdio_init_all();
    // Set up our UART
    //uart_init(UART_ID, BAUD_RATE);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    //gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    //gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);


    // Tell the tacho pin that the PWM is in charge of its value.
    gpio_set_function(Tacho_PIN, GPIO_FUNC_PWM);
    // Figure out which slice we just connected to the LED pin
    uint slice_num = pwm_gpio_to_slice_num(Tacho_PIN);

    // // Get some sensible defaults for the slice configuration. By default, the
    // // counter is allowed to wrap over its maximum range (0 to 2**16-1)
    // pwm_config config = pwm_get_default_config();
    // // Set divider, reduces counter clock to sysclock/this value
    // pwm_config_set_clkdiv(&config, 1.f);
    // // Load the configuration into our PWM slice, and set it running.
    // pwm_init(slice_num, &config, true);
    // pwm_set_gpio_level(Tacho_PIN, 0);

// determine top given Hz - assumes free-running counter rather than phase-correct
	uint32_t  f_sys = clock_get_hz(clk_sys); // typically 125'000'000 Hz
	float divider = f_sys / 1000000UL;  // let's arbitrarily choose to run pwm clock at 1MHz
	pwm_set_clkdiv(slice_num, divider); // pwm clock should now be running at 1MHz
	uint32_t top =  1000000UL/f_pwm -1; // TOP is u16 has a max of 65535, being 65536 cycles
	pwm_set_wrap(slice_num, top);

	// set duty cycle
	uint16_t level = (top+1) * duty / 100 -1; // calculate channel level from given duty cycle in %
	pwm_set_chan_level(slice_num, 0, level); 
	
	pwm_set_enabled(slice_num, true); // let's go!


    // GPIO initialisation.
    // We will make this GPIO an input, and pull it up by default
    gpio_init(GPIO);
    gpio_set_dir(GPIO, GPIO_IN);
    gpio_pull_up(GPIO);
    gpio_init(GPIO_O);
    gpio_set_dir(GPIO_O, GPIO_OUT);

    // Example of using the HW divider. The pico_divider library provides a more user friendly set of APIs 
    // over the divider (and support for 64 bit divides), and of course by default regular C language integer
    // divisions are redirected thru that library, meaning you can just use C level `/` and `%` operators and
    // gain the benefits of the fast hardware divider.
    // int32_t dividend = 123456;
    // int32_t divisor = -321;
    // // This is the recommended signed fast divider for general use.
    // divmod_result_t result = hw_divider_divmod_s32(dividend, divisor);
    // printf("%d/%d = %d remainder %d\n", dividend, divisor, to_quotient_s32(result), to_remainder_s32(result));
    // // This is the recommended unsigned fast divider for general use.
    // int32_t udividend = 123456;
    // int32_t udivisor = 321;
    // divmod_result_t uresult = hw_divider_divmod_u32(udividend, udivisor);
    // printf("%d/%d = %d remainder %d\n", udividend, udivisor, to_quotient_u32(uresult), to_remainder_u32(uresult));

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

    sleep_ms(3000); // 0.5s delay
    puts("Hello, world!");

        //Initialize interface
    can0.reset();
    can0.setBitrate(CAN_500KBPS, MCP_16MHZ);
    can0.setNormalMode();

    //     while(1){
    //     gpio_put(GPIO_O, 1); // Set pin 25 to high
    //     printf("LED ON!\n");
    //     sleep_ms(1000); // 0.5s delay

    //     gpio_put(GPIO_O, 0); // Set pin 25 to low
    //     printf("LED OFF!\n");
    //     sleep_ms(1000); // 0.5s delay
    // }





    //Listen loop
    while(true) {
        if(can0.readMessage(&rx) == MCP2515::ERROR_OK) {
            printf("New frame from ID: %10x\n", rx.can_id);
            if(rx.can_id==MEGASQUIRT_SIMPLIFIED_DASH_BROADCAST_MEGASQUIRT_DASH0_FRAME_ID)
            {
                megasquirt_simplified_dash_broadcast_megasquirt_dash0_unpack(&dash0_data, rx.data,rx.can_dlc);
                // decode RPM signal
                RPM=dash0_data.rpm;
                printf("RPM is %d\n", RPM);
            }
        }
        int16_t ch = getchar_timeout_us(100);
        if(ch != PICO_ERROR_TIMEOUT) {
           // uart_putc_raw(UART0_ID, ch);    // Send to UART0
           // uart_putc_raw(UART1_ID, ch);    // Send to UART1
           //                // Echo back so you can see what you've done
            //ch = getchar_timeout_us(100);
            printf("Got %c, val %d \n", ch, ch);
        

            switch(ch){
            case '\n':
                duty=-1;
                break;
            case 48:
                duty=0;
                break;
            case 49:
                duty=10;
                break;
            //break;
            case 50:
                duty=20;
                break;
              
            //break;
            case 51:
                duty=30;
                break;
            //break;
            case 52:
                duty=40;
                break;
            //break;
            case 53:
                duty=50;
                break;
            case 54:
                duty=60;
                break;
            case 55:
                duty=70;
                break;
            case 56:
                duty=80;
                break;            
            case 57:
                duty=90;
                break;
            case 58:
                duty=100;
                break;
            //break;
            default:
                duty=100;
            //break;
            }
            if(duty>0 & duty<=100)
            {
            	// set duty cycle
                uint16_t level = (top+1) * duty / 100 -1; // calculate channel level from given duty cycle in %
                pwm_set_chan_level(slice_num, 0, level);
                printf("PWM: %d duty and level=%d\n",duty,level);
            }
            if(duty==0){
                uint16_t level = 0; // calculate channel level from given duty cycle in %
                pwm_set_chan_level(slice_num, 0, level);
                printf("PWM: %d duty and level=%d\n",duty,level);
            }
        }
    }

    return 0;
}


