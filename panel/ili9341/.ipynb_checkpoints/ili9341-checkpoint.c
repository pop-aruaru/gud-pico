// SPDX-License-Identifier: MIT

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board.h"
#include "tusb.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"
#include "pico/unique_id.h"
#include "pico/bootrom.h"
#include "pico/stdio_uart.h"

#include "driver.h"
#include "gud.h"
#include "mipi_dbi.h"
#include "cie1931.h"

#define TFT_BLACK       0x0000      /*   0,   0,   0 */
#define TFT_BLUE        0x001F      /*   0,   0, 255 */
#define TFT_GREEN       0x07E0      /*   0, 255,   0 */
#define TFT_RED         0xF800      /* 255,   0,   0 */
#define TFT_WHITE       0xFFFF      /* 255, 255, 255 */

#define GUD_DEBUG 0

//#define LOG
//#define LOG2
//#define LOG3
#define USE_WATCHDOG    0
#define PANIC_REBOOT_BLINK_LED_MS   100

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
/*
 * 0 = led off
 * 1 = power led, off while flushing
 * 2 = on while flushing
 */
#define LED_ACTION  0

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
// ディスプレイのサイズ設定　1:2.4 , 2:2.8 , 3:3.2
#define INCH 1
#ifdef SEEED_XIAO_RP2040
#define ROTATE 0
#else
#define ROTATE 180
#endif

#define WIDTH   320
#define HEIGHT  240
#if INCH==1
#define WIDTH_MM   50
#define HEIGHT_MM  37
#elif INCH==2
#define WIDTH_MM   58
#define HEIGHT_MM  43
#elif INCH==3
#define WIDTH_MM   66
#define HEIGHT_MM  50
#endif // 実サイズ
#define BL_DEF_LEVEL    100
// There's not room for to full buffers so max_buffer_size must be set
uint16_t framebuffer[WIDTH * HEIGHT];
uint16_t compress_buf[WIDTH * 120];
static uint64_t panic_reboot_blink_time;

#ifdef SEEED_XIAO_RP2350
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
#elif 0
//#error 2350
//for xiao_rp2350
#define XPT2046_CS  27 //sikl1
#define RESET_GPIO  28 //silk2
#define BL_GPIO     6  //silk4
#define BOOTSEL     1  //silk7
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
//#error 1
//for xiao_rp2040
#define XPT2046_CS  27 //sikl1
#define RESET_GPIO  28 //silk2
//#define BL_GPIO     6  //silk4
#define BL_GPIO     1  //silk7
#define BOOTSEL     26  //silk0
#define LED_BLUE    25
#define LED_GREEN   16
static const struct mipi_dbi dbi = {
    .spi = spi0,
    .sck = 2,
    .mosi = 3,
    .cs = 29,//silk3
    .dc = 0,//silk6
    .baudrate = 64 * 1024 * 1024, // 64MHz
};
#endif
#ifdef RASPBERRYPI_PICO
//#error 2
//for pico
#define XPT2046_INT 0  //sikl0
#define XPT2046_CS  1  //sikl1
#define BOOTSEL     5  //silk5 red-switch
#define BL_GPIO     13 //silk13
#define RESET_GPIO  15 //silk15
#define LED_YELLO   25
static const struct mipi_dbi dbi = {
    .spi = spi1,
    .sck = 10,
    .mosi = 11,
    //.miso = 12,
    .cs = 9,  
    .dc = 8,
    .baudrate = 64 * 1024 * 1024, // 80MHz
};
#endif
#ifdef RASPBERRYPI_PICO2
//for pico2
#define XPT2046_INT 0  //sikl0
#define XPT2046_CS  1  //sikl1
#define BOOTSEL     5  //silk5 red-switch
#define BL_GPIO     13 //silk13
#define RESET_GPIO  15 //silk15
#define LED_YELLO   25
static const struct mipi_dbi dbi = {
    .spi = spi1,
    .sck = 10,
    .mosi = 11,
    //.miso = 12,
    .cs = 9,  
    .dc = 8,
    .baudrate = 80 * 1024 * 1024, // 80MHz
};
#endif
#ifndef BL_GPIO
#error "no board"
#endif
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
}

static void backlight_init(uint gpio)
{
    pwm_config cfg = pwm_get_default_config();
    pwm_set_wrap(pwm_gpio_to_slice_num(gpio), 65535);
    pwm_init(pwm_gpio_to_slice_num(gpio), &cfg, true);
    gpio_set_function(gpio, GPIO_FUNC_PWM);
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
    LOG3("%s: x=%u y=%u width=%u height=%u length=%u compression=0x%x\n", __func__,
         set_buf->x, set_buf->y, set_buf->width, set_buf->height, set_buf->length, set_buf->compression);

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
    uint32_t length = set_buf->length;

    LOG2("%s: x=%u y=%u width=%u height=%u length=%u compression=0x%x\n", __func__,
         set_buf->x, set_buf->y, set_buf->width, set_buf->height, set_buf->length, set_buf->compression);

    mipi_dbi_update16(&dbi, set_buf->x, set_buf->y, set_buf->width, set_buf->height, buf, length);

    if (LED_ACTION == 1)
        board_led_write(true);
    else if (LED_ACTION == 2)
        board_led_write(false);
}

static const uint8_t pixel_formats[] = {
    GUD_PIXEL_FORMAT_RGB565,
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
    .width_mm = WIDTH_MM, //58,
    .height_mm = HEIGHT_MM, //43,

    .get_serial_number = gud_display_edid_get_serial_number,
};

const struct gud_display display = {
    .width = WIDTH,
    .height = HEIGHT,

//    .flags = GUD_DISPLAY_FLAG_FULL_UPDATE,

    .compression = GUD_COMPRESSION_LZ4,// or 0
    .max_buffer_size = sizeof(compress_buf),

    .formats = pixel_formats,
    .num_formats = 1,

    .connector_properties = connector_properties,
    .num_connector_properties = 1,
#if USE_WATCHDOG
    // Tell the host to send a connector status request every 10 seconds for our tinyusb "watchdog"
    .connector_flags = GUD_CONNECTOR_FLAGS_POLL_STATUS,
#endif
    .edid = &edid,

    .controller_enable = controller_enable,
    .display_enable = display_enable,

    .state_commit = state_commit,

    .set_buffer = set_buffer,
    .write_buffer = write_buffer,
};

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
for (int i = 0; i < HEIGHT * WIDTH; i++) {
        framebuffer[i] = TFT_GREEN;
    }
    backlight_init(BL_GPIO);
    mipi_dbi_spi_init(&dbi);

    if (RESET_GPIO >= 0)
        mipi_dbi_hw_reset(RESET_GPIO);

    mipi_dbi_command(&dbi, MIPI_DCS_SOFT_RESET);

    sleep_ms(150); // ????

    mipi_dbi_command(&dbi, MIPI_DCS_SET_DISPLAY_OFF);

    mipi_dbi_command(&dbi, ILI9341_PWCTRLB, 0x00, 0x83, 0x30);
    mipi_dbi_command(&dbi, ILI9341_PWRSEQ, 0x64, 0x03, 0x12, 0x81);
    mipi_dbi_command(&dbi, ILI9341_DTCTRLA, 0x85, 0x01, 0x79);
    mipi_dbi_command(&dbi, ILI9341_PWCTRLA, 0x39, 0x2c, 0x00, 0x34, 0x02);
    mipi_dbi_command(&dbi, ILI9341_PUMPCTRL, 0x20);
    mipi_dbi_command(&dbi, ILI9341_DTCTRLB, 0x00, 0x00);

    /* Power Control */
    mipi_dbi_command(&dbi, ILI9341_PWCTRL1, 0x26);
    mipi_dbi_command(&dbi, ILI9341_PWCTRL2, 0x11);
    /* VCOM */
    mipi_dbi_command(&dbi, ILI9341_VMCTRL1, 0x35, 0x3e);
    mipi_dbi_command(&dbi, ILI9341_VMCTRL2, 0xbe);

    /* Memory Access Control */
    mipi_dbi_command(&dbi, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FORMAT_16BIT);

    /* Frame Rate */
    mipi_dbi_command(&dbi, ILI9341_FRMCTR1, 0x00, 0x1b);

    /* Gamma */
    mipi_dbi_command(&dbi, ILI9341_EN3GAM, 0x08);
    mipi_dbi_command(&dbi, MIPI_DCS_SET_GAMMA_CURVE, 0x01); //0x26
    mipi_dbi_command(&dbi, ILI9341_PGAMCTRL,                //0xe0
                     0x1f, 0x1a, 0x18, 0x0a, 0x0f, 0x06, 0x45, 0x87,
                     0x32, 0x0a, 0x07, 0x02, 0x07, 0x05, 0x00);
    mipi_dbi_command(&dbi, ILI9341_NGAMCTRL,                //0xe1
                     0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3a, 0x78,
                     0x4d, 0x05, 0x18, 0x0d, 0x38, 0x3a, 0x1f);

    /* DDRAM */
    mipi_dbi_command(&dbi, ILI9341_ETMOD, 0x07);

    /* Display */
    mipi_dbi_command(&dbi, ILI9341_DISCTRL, 0x0a, 0x82, 0x27, 0x00);
    mipi_dbi_command(&dbi, MIPI_DCS_EXIT_SLEEP_MODE);
    sleep_ms(100);

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
    addr_mode |= ILI9341_MADCTL_BGR;
    mipi_dbi_command(&dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);


    mipi_dbi_command(&dbi, MIPI_DCS_SET_DISPLAY_ON);

    LOG("LCD clear start.\n\r");
    backlight_set(100);
    for (int color=0;color<4;color+=1){
        uint16_t *pos=(uint16_t*)framebuffer;
        for(int x=0;x<WIDTH;x++){
            for (int y=0;y<HEIGHT;y++){
                if(color==0)*pos=0xf800;// Red
                else if(color==1)*pos=0x07e0;// Green
                else if(color==2)*pos=0x001f;// Blue
                else *pos=0x0000;//Black
                pos++;
            }
        }
        // Clear display
        mipi_dbi_update16(&dbi, 0, 0, WIDTH, HEIGHT, framebuffer, WIDTH * HEIGHT * 2);
        sleep_ms(500);
    }
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
/*
    "LEVEL_LOW",  // 0x1
    "LEVEL_HIGH", // 0x2
    "EDGE_FALL",  // 0x4
    "EDGE_RISE"   // 0x8
*/
int main(void)
{
    //#ifdef RASPBERRYPI_PICO2 
    #if GUD_DEBUG >= 1
    stdio_uart_init_full(uart0,
        PICO_DEFAULT_UART_BAUD_RATE,PICO_DEFAULT_UART_TX_PIN,PICO_DEFAULT_UART_RX_PIN);
    sleep_ms(100);
    LOG("main");
    LOG("printf main start\n\r\n\r");
    {
        int a;
        a=sizeof(compress_buf);
        LOG3("sizeof(compress_buf): %ld\n\r",a);
        a=sizeof(*framebuffer);
        LOG3("sizeof(framebuffer): %ld\n\r",a);
    }
    LOG("PICO_FLASH_SIZE_BYTES = %dKbyte\n",PICO_FLASH_SIZE_BYTES/1024);

    #endif

    board_init();

    if (LED_ACTION)
        LOG("LED action\n");
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
