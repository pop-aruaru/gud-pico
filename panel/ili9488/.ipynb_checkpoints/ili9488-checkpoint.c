// SPDX-License-Identifier: MIT

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board.h"
#include "tusb.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "pico/unique_id.h"
#include "pico/bootrom.h"
#include "pico/stdio_uart.h"

#include "driver.h"
#include "gud.h"
#include "mipi_dbi.h"
#include "cie1931.h"
#if 0
#define TFT_BLACK       0x000000      /*   0,   0,   0 */
#define TFT_BLUE        0x0000ff      /*   0,   0, 255 */
#define TFT_GREEN       0x00FF00      /*   0, 255,   0 */
#define TFT_RED         0xFF0000      /* 255,   0,   0 */
#define TFT_WHITE       0xFFFFFF      /* 255, 255, 255 */
#endif
#define GUD_DEBUG 1
//#define LOG
//#define LOG2
//#define LOG3
#define USE_WATCHDOG    0
#define PANIC_REBOOT_BLINK_LED_MS   100
#if 1
#if GUD_DEBUG >= 1
#define LOG     printf
#else
#define LOG
#endif
#if GUD_DEBUG >= 2
#define LOG2    printf
#else
#define LOG2
#endif
#if GUD_DEBUG >= 3
#define LOG3    printf
#else
#define LOG3
#endif
#endif
/*
 * 0 = led off
 * 1 = power led, off while flushing
 * 2 = on while flushing
 */
#define LED_ACTION  2

/*
 * Pins are mapped for the Watterott RPi-Display on a Pico HAT Expansion board:
 * https://shop.sb-components.co.uk/products/raspberry-pi-pico-hat-expansion
 *
 * Pi           Pico    Function
 * -----------------------------
 * GPIO11       GP2     SPI CLK
 * GPIO10       GP3     SPI MOSI
 * GPIO9        GP4     SPI MISO
 * GPIO8        GP5     SPI CE0 - MI0283QT
 * GPIO7        GP19    SPI CE1 - ADS7846
 * GPIO23       GP27    MI0283QT /reset
 * GPIO24       GP26    MI0283QT D/C
 * GPIO18       GP28    MI0283QT backlight
 * GPIO25       GP22    ADS7846 /PENIRQ
 *
 *
 * The Adafruit PiTFT 2.8" should also work (not tested):
 * - Doesn't have a reset pin
 * - MISO is not connected to the display controller
 * - STMPE610 touch controller, also used to control backlight (no pwm)
 *
 * GPIO25       D/C
 * GPIO24       STMPE610 INT
 */
#ifdef SEEED_XIAO_RP2350
//#error 2350
//for xiao_rp2350
#define XPT2046_CS  27 //sikl1
#define RESET_GPIO  28 //silk2
#define BL_GPIO     1  //silk7
#define BOOTSEL     26  //silk0
#define LED_YELLO   25
static const struct mipi_dbi dbi = {
    .spi = spi0,
    .sck = 2,
    .mosi = 3,
    .cs = 5,//silk3
    .dc = 0,//silk6
    .baudrate = 64 * 1024 * 1024, // 64MHz
};
#endif
#ifdef SEEED_XIAO_RP2040
//#error 2040
//for xiao_rp2040
#define XPT2046_CS  27 //sikl1
#define RESET_GPIO  28 //silk2
#define BL_GPIO     1  //silk7
#define BOOTSEL     26  //silk0
#define LED_YELLO   25
static const struct mipi_dbi dbi = {
    .spi = spi0,
    .sck = 2,
    .mosi = 3,
    .cs = 5,//silk3
    .dc = 0,//silk6
    .baudrate = 64 * 1024 * 1024, // 64MHz
};
#endif
#ifdef RASPBERRYPI_PICO2
//#error 2
//for pico2
#define XPT2046_INT 3  //sikl0
#define XPT2046_CS  4  //sikl1
#define RESET_GPIO  15 //silk15
#define BL_GPIO     13 //silk13
#define PICO_DEFAUL_UART_BAUD_RATE 1000000
#define BOOTSEL     5
#define LED_YELLO   25
static const struct mipi_dbi dbi = {
    .spi = spi1,
    .sck = 10,
    .mosi = 11,
    //.miso = 12,
    .cs = 9,  
    .dc = 8,
    .baudrate = 37.5 * 1024 * 1024, // 37.5MHz
};
#endif
#ifdef RASPBERRYPI_PICO
//#error 2
//for pico
#define XPT2046_INT 3  //sikl0
#define XPT2046_CS  4  //sikl1
#define RESET_GPIO  15 //silk15
#define BL_GPIO     13 //silk13
#define PICO_DEFAUL_UART_BAUD_RATE 1000000
#define BOOTSEL     5
#define LED_YELLO   25
static const struct mipi_dbi dbi = {
    .spi = spi1,
    .sck = 10,
    .mosi = 11,
    //.miso = 12,
    .cs = 9,  
    .dc = 8,
    .baudrate = 60 * 1024 * 1024, // 80MHz
};
#error "momory error"
#endif

#ifndef BL_GPIO
/*
cmake -DPICO_BOARD=seeed_xiao_rp2350
cmake -DPICO_BOARD=pico2
cmake -DPICO_BOARD=pico
cmake -DPICO_BOARD=seeed_xiao_rp2040
*/
#error "no define board"
#error "cmake -DPICO_BOARD=raspberypi_pico2"
#error "cmake -DPICO_BOARD=seeed_xiao_rp2350"
#endif

// There's not room for to full buffers so max_buffer_size must be set
/*
縦：ROTATE:90 、横：ROTATE=180
色：　1：RGB565、2：RGB666、3：RGB888 RGB666とRGB888は表示は24bit
ディスプレイのサイズ設定　1:3.5 , 2:4
*/
#define INCH 1
#define VGA
#define COLOR_MODE 1
#define BL_DEF_LEVEL    100
#define COMPRESS 1


#ifdef VGA
#define WIDTH   480
#define HEIGHT  320
#if defined(SEEED_XIAO_RP2350)
#define ROTATE 180
#else
#define ROTATE 0
#endif
#elifdef VGA_H
#define WIDTH   320
#define HEIGHT  480
#define ROTATE 90
#else
#define WIDTH   480
#define HEIGHT  320
#define ROTATE 180
#endif
#if COLOR_MODE==1
    #define RGB565
#elif COLOR_MODE==2
    #define RGB666
#elif COLOR_MODE==3
    #define RGB888
#endif
#if INCH==1
#define WIDTH_MM   70
#define HEIGHT_MM  50
#elif INCH==2
#define WIDTH_MM   83
#define HEIGHT_MM  58
#endif // 実サイズ


#define BL_DEF_LEVEL    100
#ifdef RGB565
uint16_t framebuffer[WIDTH * HEIGHT ];
#elif defined(RGB666)
uint8_t framebuffer[WIDTH * HEIGHT * 3];
#elif defined(RGB888)
uint8_t framebuffer[WIDTH * HEIGHT * 3];
#endif
//uint8_t *framebuffer;

#ifdef RGB565
    #if COMPRESS
    uint16_t compress_buf[WIDTH * 120];
    #else
    uint16_t *compress_buf;
    #endif
#else
    #if COMPRESS
    uint16_t compress_buf[WIDTH * 50];
    #else
    uint16_t *compress_buf;
    #endif
#endif
static uint64_t panic_reboot_blink_time;
static uint brightness = BL_DEF_LEVEL;

static void backlight_set(int level)
{
    uint16_t pwm;

    LOG("Set backlight: %d\n", level);
    if (level > 100)
        return;

    if (level < 0)
        pwm = 0;
    else
        pwm = cie1931[level];

    pwm_set_gpio_level(BL_GPIO, pwm);
    #ifdef LED_YELLO
    pwm_set_gpio_level(LED_YELLO, pwm);
    #endif
}

static void backlight_init(uint gpio)
{
    pwm_config cfg = pwm_get_default_config();
    pwm_set_wrap(pwm_gpio_to_slice_num(gpio), 65535);
    pwm_init(pwm_gpio_to_slice_num(gpio), &cfg, true);
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    #ifdef LED_YELLO
    pwm_set_wrap(pwm_gpio_to_slice_num(LED_YELLO), 65535);
    pwm_init(pwm_gpio_to_slice_num(LED_YELLO), &cfg, true);
    gpio_set_function(LED_YELLO, GPIO_FUNC_PWM);
    #endif
}

static int controller_enable(const struct gud_display *disp, uint8_t enable)
{
    LOG("%s: enable=%u\n", __func__, enable);
    return 0;
}

static int display_enable(const struct gud_display *disp, uint8_t enable)
{
    LOG("%s: enable=%u\n", __func__, enable);

    if (enable)
        backlight_set(brightness);
    else
        backlight_set(-1);

    return 0;
}

static int state_commit(const struct gud_display *disp, const struct gud_state_req *state, uint8_t num_properties)
{
    LOG("%s: mode=%ux%u format=0x%02x connector=%u num_properties=%u\n",
        __func__, state->mode.hdisplay, state->mode.vdisplay, state->format, state->connector, num_properties);

    for (uint8_t i = 0; i < num_properties; i++) {
        const struct gud_property_req *prop = &state->properties[i];
        LOG("  prop=%u val=%llu\n", prop->prop, prop->val);
        switch (prop->prop) {
            case GUD_PROPERTY_BACKLIGHT_BRIGHTNESS:
                brightness = prop->val;
                backlight_set(brightness);
                break;
            default:
                LOG("Unknown property: %u\n", prop->prop);
                break;
        };
    }

    return 0;
}

static int set_buffer(const struct gud_display *disp, const struct gud_set_buffer_req *set_buf)
{
    //LOG3("%s: x=%u y=%u width=%u height=%u length=%u compression=0x%x\n", __func__,
    //     set_buf->x, set_buf->y, set_buf->width, set_buf->height, set_buf->length, set_buf->compression);

    // Wait for SPI transfer to finish so we don't clobber the buffer
    mipi_dbi_update_wait(&dbi);

    if (LED_ACTION == 1)
        board_led_write(false);
    else if (LED_ACTION == 2)
        board_led_write(true);

    return 0;
}

static void write_buffer(const struct gud_display *disp, const struct gud_set_buffer_req *set_buf, void *buf)
{
    LOG2("%s: x=%u y=%u width=%u height=%u length=%u compression=0x%x\n", __func__,
         set_buf->x, set_buf->y, set_buf->width, set_buf->height,
        set_buf->length, set_buf->compression);
    uint32_t length = set_buf->length;
    #ifdef RGB565
    mipi_dbi_update16_24(&dbi, set_buf->x, set_buf->y, set_buf->width, set_buf->height, buf, length);
    #else
    mipi_dbi_update24(&dbi, set_buf->x, set_buf->y, set_buf->width, set_buf->height, buf, length);
    #endif
    if (LED_ACTION == 1)
        board_led_write(true);
    else if (LED_ACTION == 2)
        board_led_write(false);
}

static const uint8_t pixel_formats[] = {
#ifdef RGB565
    GUD_PIXEL_FORMAT_RGB565,
#elif defined(RGB666)
    GUD_PIXEL_FORMAT_RGB888,
#elif defined(RGB888)
    GUD_PIXEL_FORMAT_RGB888,
#endif
};

static const struct gud_property_req connector_properties[] = {
    {
        .prop = GUD_PROPERTY_BACKLIGHT_BRIGHTNESS,
        .val = BL_DEF_LEVEL,
    },
};

static uint32_t gud_display_edid_get_serial_number(void)
{
    pico_unique_board_id_t id_out;

    pico_get_unique_board_id(&id_out);
    return *((uint64_t*)(id_out.id));
}

static const struct gud_display_edid edid = {
    .name = "RPi-Display",
    .pnp = "WAT",
    .product_code = 0x01,
    .year = 2025,
    .width_mm = WIDTH_MM,
    .height_mm = HEIGHT_MM,

    .get_serial_number = gud_display_edid_get_serial_number,
};

const struct gud_display display = {
    .width = WIDTH,
    .height = HEIGHT,

//    .flags = GUD_DISPLAY_FLAG_FULL_UPDATE,

#if COMPRESS
    .compression = GUD_COMPRESSION_LZ4,
    .max_buffer_size = sizeof(compress_buf),
#else
    .compression = 0,
    .max_buffer_size = 0,
#endif

    .formats = pixel_formats,
    .num_formats = 1,

    .connector_properties = connector_properties,
    .num_connector_properties = 1,

    .edid = &edid,

    .controller_enable = controller_enable,
    .display_enable = display_enable,

    .state_commit = state_commit,

    .set_buffer = set_buffer,
    .write_buffer = write_buffer,
};

#define ILI9488_COLSET  0x2A
#define ILI9488_RAWSET  0x2B
#define ILI9488_RAMWR   0x2C
#define ILI9488_MADCTL  0x36
#define ILI9488_MADCTL_RGB 0x08
#define ILI9488_INVOFF  0x20
#define ILI9488_INVON   0x21

#define ILI9341_FRMCTR1         0xb1
#define ILI9341_DISCTRL         0xb6
#define ILI9341_ETMOD           0xb7

#define ILI9341_PWCTRL1         0xc0
#define ILI9341_PWCTRL2         0xc1
#define ILI9341_VMCTRL1         0xc5
#define ILI9341_VMCTRL2         0xc7
#define ILI9341_PWCTRLA         0xcb
#define ILI9341_PWCTRLB         0xcf

#define ILI9341_PGAMCTRL        0xe0
#define ILI9341_NGAMCTRL        0xe1
#define ILI9341_DTCTRLA         0xe8
#define ILI9341_DTCTRLB         0xea
#define ILI9341_PWRSEQ          0xed

#define ILI9341_EN3GAM          0xf2
#define ILI9341_PUMPCTRL        0xf7

#define ILI9341_MADCTL_BGR      BIT(3)
#define ILI9341_MADCTL_MV       BIT(5)
#define ILI9341_MADCTL_MX       BIT(6)
#define ILI9341_MADCTL_MY       BIT(7)

static void init_display(void)
{
    backlight_init(BL_GPIO);
    mipi_dbi_spi_init(&dbi);

    if (RESET_GPIO >= 0)
        mipi_dbi_hw_reset(RESET_GPIO);

    mipi_dbi_command(&dbi, MIPI_DCS_SOFT_RESET);

    sleep_ms(150); // ????

        mipi_dbi_command(&dbi,0xF7,0xA9,0x51,0X2C,0X82);
        mipi_dbi_command(&dbi,0xC0,0x0F,0x0f);
        //#//VGH = 5*VCI   VGL = -3*VCI
        mipi_dbi_command(&dbi,0xC1,0x47);       
        mipi_dbi_command(&dbi,0xC5,0x00,0x4D,0x80);      
        mipi_dbi_command(&dbi,0xB1,0xB0,0X11);
        //#//mipi_dbi_command(&dbi,0xB1,0xA0);
        mipi_dbi_command(&dbi,0xB4,0x02);
        mipi_dbi_command(&dbi,0x36,0x48);
        //f=(display_px<<4)+transfer_px
        #if defined(RGB565)
        mipi_dbi_command(&dbi,0x3A,0x66);//#  6:RGB666 , 7:RGB888
        #elif defined(RGB666)
        mipi_dbi_command(&dbi,0x3A,0x66);//#  6:RGB666 , 7:RGB888
        #elif defined(RGB888)
        mipi_dbi_command(&dbi,0x3A,0x66);//#  6:RGB666 , 7:RGB888
        #endif
        mipi_dbi_command(&dbi,0x21,0x00);//# //IPS 
        mipi_dbi_command(&dbi,0xE9,0x00);
        mipi_dbi_command(&dbi,0xF7,0xA9,0x51,0x2C,0x82);
        mipi_dbi_command(&dbi,0xE0,0x00,0x07,0x0B,0x03,0x0F,0x05,0x30,0x56,0x47,0x04,0x0B,0x0A,0x2D,0x37,0x0F);
        mipi_dbi_command(&dbi,0xE1,0x00,0x0E,0x13,0x04,0x11,0x07,0x39,0x45,0x50,0x07,0x10,0x0D,0x32,0x36,0x0F);
        mipi_dbi_command(&dbi,0x11);
        sleep_ms(400);  
        mipi_dbi_command(&dbi,0x29);//5.2.21. Display ON (29h) 
        sleep_ms(100);
    if(1){
        uint16_t rotation = ROTATE;
        uint8_t addr_mode;
    
        switch (rotation) {
        default:
            addr_mode = ILI9341_MADCTL_MV | ILI9341_MADCTL_MY |
                    ILI9341_MADCTL_MX;
            break;
        case 90:
            addr_mode = ILI9341_MADCTL_MY;
            break;
        case 180:
            addr_mode = ILI9341_MADCTL_MV;
            break;
        case 270:
            addr_mode = ILI9341_MADCTL_MX;
            break;
        }
        //addr_mode |= ILI9341_MADCTL_BGR;
        mipi_dbi_command(&dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);
    }

    mipi_dbi_command(&dbi, MIPI_DCS_SET_DISPLAY_ON);//5.2.21. Display ON (29h) 
    //mipi_dbi_command(&dbi, 0xf7,0xA9,0x51,0x2C,0x82);//ILI9341_PUMPCTRL
    sleep_ms(100);
    // Clear display
    
    #ifdef RGB565
        /* RGB565 */
        uint16_t * pos;
        backlight_set(100);
        for (int color=0;color<3;color+=1){
            LOG("color:%d\n\r",color);
            pos=framebuffer;
            for(int x=0;x<WIDTH;x++){
                for (int y=0;y<HEIGHT;y++){
                    if (color==0){
                        *pos++=0xf800;// Red
                    }else if(color==1){
                        *pos++=0x07e0;// Green
                    }else if(color==2){
                        *pos++=0x001f;// Blue
                    }else{
                        *pos++=0xffff;// White
                    }
                }
            }
            mipi_dbi_update16_24(&dbi, 0, 0, WIDTH, HEIGHT, framebuffer, WIDTH * HEIGHT*2 );
            //mipi_dbi_update24(&dbi, 0, 0, WIDTH, HEIGHT, framebuffer, WIDTH * HEIGHT*2 );
            sleep_ms(100);
        }
        #if 1
        // Clear display
        LOG("LCD clear start.\n\r");
        pos=(uint16_t*)framebuffer;
        for(int x=0;x<WIDTH;x++){
            for (int y=0;y<HEIGHT;y++){
                //*pos=0xf800;// Blue
                //*pos=0x07e0;// Green
                //*pos=0x001f;// Blue
                *pos=0x0000;//Black
                pos++;
            }
        }
        mipi_dbi_update16_24(&dbi, 0, 0, WIDTH, HEIGHT, framebuffer, WIDTH * HEIGHT * 2);
        sleep_ms(2000);
        //mipi_dbi_update24(&dbi, 0, 0, WIDTH, HEIGHT, framebuffer, WIDTH * HEIGHT * 2);
        #endif
        LOG("LCD INIT finish!\n\r");
    #else // RB565以外
        #if 0 // test pattern
        uint8_t * pos;
        backlight_set(100);
        for (int color=4;color<256;color+=4){
            pos=framebuffer;
            for(int x=0;x<WIDTH;x++){
                for (int y=0;y<HEIGHT;y++){
                    if (y<256){
                        *pos++=255-y;
                        *(pos++)=x>>1;
                        *(pos++)=y>>1;
                    }else{
                        *pos++=0;
                        *(pos++)=x>>1;
                        *(pos++)=y>>1;
                    }
                }
            }
            mipi_dbi_update24(&dbi, 0, 0, WIDTH, HEIGHT, framebuffer, WIDTH * HEIGHT *3);
            //printf("color:%d\n\r",color);
        }
        #elif 1
        uint8_t * pos;
        backlight_set(100);
        for (int color=0;color<3;color+=1){
            pos=framebuffer;
            for(int x=0;x<WIDTH;x++){
                for (int y=0;y<HEIGHT;y++){
                    if (color==0){
                        *pos++=255;// R
                        *pos++=0;
                        *pos++=0;
                    }else if(color==1){
                        *pos++=0;
                        *pos++=255;// G
                        *pos++=0;
                    }else{
                        *pos++=0;
                        *pos++=0;
                        *pos++=255;// B
                    }
                }
            }
            mipi_dbi_update24(&dbi, 0, 0, WIDTH, HEIGHT, framebuffer, WIDTH * HEIGHT*3 );
            //printf("color:%d\n\r",color);
            sleep_ms(1000);
        }
        #endif //test pattern
        #if 1
        {
            uint8_t * pos;
            pos=framebuffer;
            for(int x=0;x<WIDTH;x++){
                for (int y=0;y<HEIGHT;y++){
                    *pos++=0;
                    *(pos++)=0;
                    *(pos++)=0;
                }
            }
        }
        #endif
        mipi_dbi_update24(&dbi, 0, 0, WIDTH, HEIGHT, framebuffer, WIDTH * HEIGHT * 3 );
    #endif //RGB888 or RGB666
    backlight_set(0);
}
void gpio_callback(uint gpio, uint32_t events) {
    // Put the GPIO event(s) that just happened into event_str
    if (events & 1) {
        // Copy this event string into the user string
        #ifdef LED_BLUE
        gpio_put(LED_BLUE, 0);
        #endif
    }
    if (events & 2) {
        // Copy this event string into the user string
        #ifdef LED_BLUE
        gpio_put(LED_BLUE, 1);
        #endif
    }
    #ifdef SEEED_XIAO_RP2350
    if (events == BOOTSEL) {
        // Copy this event string into the user string
        #ifdef LED_GREEN
        gpio_put(LED_GREEN, 0);
        #endif
        #ifdef LED_BLUE
        gpio_put(LED_BLUE, 0);
        #endif
        #ifdef LED_YELLO
        gpio_put(LED_YELLO, 0);
        #endif
         rom_reset_usb_boot ( 0,  0);
    }
    #endif
    if (events & 4) {
        // Copy this event string into the user string
        #ifdef LED_GREEN
        gpio_put(LED_GREEN, 0);
        #endif
        #ifdef LED_BLUE
        gpio_put(LED_BLUE, 0);
        #endif
        #ifdef LED_YELLO
        gpio_put(LED_YELLO, 0);
        #endif
         rom_reset_usb_boot ( 0,  0);
    }
    if (events & 8) {
        // Copy this event string into the user string
        #ifdef LED_GREEN
        gpio_put(LED_GREEN, 1);
        #endif
        #ifdef LED_BLUE
        gpio_put(LED_BLUE, 1);
        #endif
        #ifdef LED_YELLO
        gpio_put(LED_YELLO, 1);
        #endif
    }
    
}
int main(void)
{
/*    gpio_set_function(0, UART_FUNCSEL_NUM(uart0, 0));
    gpio_set_function(1, UART_FUNCSEL_NUM(uart0, 1));
    uart_init(uart0, PICO_DEFAULT_UART_BAUD_RATE);
    sleep_ms(100);
    uart_puts(uart0, "put main start\n\r\n\r");
*/    //stdio_uart_init_full();
    #ifdef RASPBERRYPI_PICO2
        #if GUD_DEBUG >= 0
        stdio_uart_init_full(uart0,
            PICO_DEFAULT_UART_BAUD_RATE,PICO_DEFAULT_UART_TX_PIN,PICO_DEFAULT_UART_RX_PIN);
        sleep_ms(100);
        LOG("main");
        printf("printf main start\n\r\n\r");
        {
            int a;
            a=sizeof(compress_buf);
            LOG3("sizeof(compress_buf): %ld\n\r",a);
            a=sizeof(*framebuffer);
            LOG3("sizeof(framebuffer): %ld\n\r",a);
        }
        LOG("PICO_FLASH_SIZE_BYTES = %dKbyte\n",PICO_FLASH_SIZE_BYTES/1024);
        LOG("SPI clk speed : %d kHz\n\r",dbi.baudrate/1024);
            LOG("PICO_FLASH_SIZE_BYTES = %dKbyte\n",PICO_FLASH_SIZE_BYTES/1024);
            LOG(" INCH : %d\n",INCH);
            LOG("COLOR_MODE : %d\n", COLOR_MODE );
            LOG("COMPRESS : %d\n",COMPRESS );
        #endif
    #endif
    if (LED_ACTION)
        board_led_write(true);

    // Pull ADS7846 CE1 high
    #ifdef XPT2046_CS
        LOG("XPT2046_CS\n");
    gpio_set_function(XPT2046_CS, GPIO_FUNC_SIO);
    gpio_set_dir(XPT2046_CS, GPIO_OUT);
    gpio_put(XPT2046_CS, 1);
    #endif
    #ifdef BOOTSEL
        LOG("BOOTSEL\n");
    gpio_init(BOOTSEL);
    gpio_pull_up(BOOTSEL); // pull it up even more!
    gpio_set_irq_enabled_with_callback(BOOTSEL, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    #endif
    #ifdef LED_BLUE
        LOG("LED_BLUE\n");
    gpio_init(LED_BLUE);
    gpio_set_dir(LED_BLUE,GPIO_OUT);
    gpio_put(LED_BLUE, 1);
    #endif
    #ifdef LED_GREEN
        LOG("LED_GREEN\n");
    gpio_init(LED_GREEN);
    gpio_set_dir(LED_GREEN,GPIO_OUT);
    gpio_put(LED_GREEN, 1);
    #endif
    #ifdef LED_YELLO
        LOG("LED_YELLO\n");
    gpio_init(LED_YELLO);
    gpio_set_dir(LED_YELLO,GPIO_OUT);
    gpio_put(LED_YELLO, 1);
    #endif
    //framebuffer = malloc(WIDTH * HEIGHT *3);

    init_display();

    gud_driver_setup(&display, framebuffer, compress_buf);

    tusb_init();

    LOG("\n\n%s: CFG_TUSB_DEBUG=%d\n", __func__, CFG_TUSB_DEBUG);

    while (1)
    {
        tud_task(); // tinyusb device task
        if (USE_WATCHDOG) {
            watchdog_update();

            uint64_t now = time_us_64();
            if (PANIC_REBOOT_BLINK_LED_MS && panic_reboot_blink_time && panic_reboot_blink_time < now) {
                static bool led_state;
                board_led_write(led_state);
                led_state = !led_state;
                panic_reboot_blink_time = now + (PANIC_REBOOT_BLINK_LED_MS * 1000);
            }

            // Sometimes we stop receiving USB requests, but the host thinks everything is fine.
            // Reset if we haven't heard from tinyusb, the host sends connector status requests every 10 seconds
            // Let the watchdog do the reset
            if (gud_driver_req_timeout(15))
                panic("Request TIMEOUT");
        }
    }

    return 0;

}

void tud_mount_cb(void)
{
    LOG("%s:\n", __func__);
    if (LED_ACTION == 2)
        board_led_write(false);
}
