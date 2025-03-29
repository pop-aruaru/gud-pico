// SPDX-License-Identifier: CC0-1.0

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board.h"
#include "tusb.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"
#include "pico/unique_id.h"

#include "driver.h"
#include "gud.h"
#include "mipi_dbi.h"
#include "cie1931.h"


#define TFT_BLACK       0x0000      /*   0,   0,   0 */
#define TFT_BLUE        0x001F      /*   0,   0, 255 */
#define TFT_GREEN       0x07E0      /*   0, 255,   0 */
#define TFT_RED         0xF800      /* 255,   0,   0 */
#define TFT_WHITE       0xFFFF      /* 255, 255, 255 */

#define LCD_PICO_DISPLAY    1
#define LCD_PICO_DISPLAY_2  2
#define LCD_WAVESHARE28     3

#ifndef LCD_TARGET
#define LCD_TARGET  LCD_WAVESHARE28
#endif

#define BL_ALWAY_ON 1
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
#define LED_ACTION  2

#define BL_GPIO LCD_BL
#define BL_DEF_LEVEL 100

#if (LCD_TARGET == LCD_PICO_DISPLAY)
#define LCD_SPI     spi0
#define LCD_MOSI    19
#define LCD_SCLK    18
#define LCD_CS      17
#define LCD_DC      16
#define LCD_BL      20
#define LCD_RESET   23
#define WIDTH   240
#define HEIGHT  135
#define START_X 40
#define START_Y 53

#define LCD_LED_R   6
#define LCD_LED_G   7
#define LCD_LED_B   8

static uint16_t compress_buf[WIDTH * HEIGHT];
static uint16_t buffer_test[WIDTH * HEIGHT];

#elif (LCD_TARGET == LCD_PICO_DISPLAY_2)
#define LCD_SPI     spi0
#define LCD_MOSI    19
#define LCD_SCLK    18
#define LCD_CS      17
#define LCD_DC      16
#define LCD_BL      20
#define LCD_RESET   23
#define WIDTH   320
#define HEIGHT  240
#define START_X 0
#define START_Y 0

// There's not room for to full buffers so max_buffer_size must be set
uint16_t compress_buf[WIDTH * 120];

#elif (LCD_TARGET == LCD_WAVESHARE28)
#define LCD_SPI     spi1
#define LCD_MOSI    11
#define LCD_SCLK    10
#define LCD_CS      9
#define LCD_DC      8
#define LCD_BL      13
#define LCD_RESET   15
#define WIDTH   320
#define HEIGHT  240
#define START_X 0
#define START_Y 0
#define XPT2046_CS  1  //16

// There's not room for to full buffers so max_buffer_size must be set
uint16_t compress_buf[WIDTH * 120];

#endif

static uint16_t framebuffer[WIDTH * HEIGHT];

static bool display_enabled;
static uint64_t panic_reboot_blink_time;

static const struct mipi_dbi dbi = {
    .spi = LCD_SPI,
    .sck = LCD_SCLK,
    .mosi = LCD_MOSI,
    .cs = LCD_CS,
    .dc = LCD_DC,
    .baudrate = 64 * 1024 * 1024, // 64MHz
};

static uint brightness = BL_DEF_LEVEL;

static void backlight_set(int level)
{
    uint16_t pwm;

    LOG("Set backlight: %d\n", level);
    if (level > 100)
        return;

#if BL_ALWAY_ON
    if (level < 0)
        level = 50;
    else
        level = 100;
#endif
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

    display_enabled = enable;

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
                if (display_enabled)
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

    mipi_dbi_update_wait(&dbi);

    if (LED_ACTION == 1)
        board_led_write(false);
    else if (LED_ACTION == 2)
        board_led_write(true);

    return 0;
}

#if LCD_TARGET == LCD_PICO_DISPLAY
static size_t r1_to_rgb565(uint16_t *dst, uint8_t *src, uint16_t src_width, uint16_t src_height)
{
    uint8_t val = 0;
    size_t len = 0;

    for (uint16_t y = 0; y < src_height; y++) {
        for (uint16_t x = 0; x < src_width; x++) {
            if (!(x % 8))
                val = *src++;
            *dst++ = val & 0x80 ? 0xffffffff : 0;
            len += sizeof(*dst);
            val <<= 1;
        }
   }

   return len;
}

static size_t rgb111_to_rgb565(uint16_t *dst, uint8_t *src, uint16_t src_width, uint16_t src_height)
{
    uint8_t rgb111, val = 0;
    size_t len = 0;

    for (uint16_t y = 0; y < src_height; y++) {
        for (uint16_t x = 0; x < src_width; x++) {
            if (!(x % 2))
                val = *src++;
            rgb111 = val >> 4;
            *dst++ = ((rgb111 & 0x04) << 13) | ((rgb111 & 0x02) << 9) | ((rgb111 & 0x01) << 4);
            len += sizeof(*dst);
            val <<= 4;
        }
    }

   return len;
}
#endif

static void write_buffer(const struct gud_display *disp, const struct gud_set_buffer_req *set_buf, void *buf)
{
    uint32_t length = set_buf->length;

    LOG2("%s: x=%u y=%u width=%u height=%u length=%u compression=0x%x\n", __func__,
         set_buf->x, set_buf->y, set_buf->width, set_buf->height, set_buf->length, set_buf->compression);

#if LCD_TARGET == LCD_PICO_DISPLAY
    if (disp->formats[0] == GUD_PIXEL_FORMAT_R1) {
        length = r1_to_rgb565(buffer_test, buf, set_buf->width, set_buf->height);
        buf = buffer_test;
    } else if (disp->formats[0] == GUD_PIXEL_FORMAT_XRGB1111) {
        length = rgb111_to_rgb565(buffer_test, buf, set_buf->width, set_buf->height);
        buf = buffer_test;
    }
#endif
    mipi_dbi_update16(&dbi, set_buf->x + START_X, set_buf->y + START_Y, set_buf->width, set_buf->height, buf, length);

    if (LED_ACTION == 1)
        board_led_write(true);
    else if (LED_ACTION == 2)
        board_led_write(false);
}

static const uint8_t pixel_formats[] = {
//    GUD_PIXEL_FORMAT_R1,
//    GUD_PIXEL_FORMAT_XRGB1111,
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
#if LCD_TARGET == LCD_PICO_DISPLAY
    .name = "pico display",
    .pnp = "PIM",
    .product_code = 0x01,
    .year = 2021,
    .width_mm = 27,
    .height_mm = 16,
#elif LCD_TARGET == LCD_PICO_DISPLAY_2
    .name = "pico display 2",
    .pnp = "PIM",
    .product_code = 0x01,
    .year = 2021,
    .width_mm = 58,
    .height_mm = 43,
#elif LCD_TARGET == LCD_WAVESHARE28
    .name = "waveshare 2.8",
    .pnp = "PIM",
    .product_code = 0x01,
    .year = 2021,
    .width_mm = 58,
    .height_mm = 43,
#endif

    .get_serial_number = gud_display_edid_get_serial_number,
};

const struct gud_display display = {
    .width = WIDTH,
    .height = HEIGHT,

//    .flags = GUD_DISPLAY_FLAG_FULL_UPDATE,

    .compression = GUD_COMPRESSION_LZ4,
#if (LCD_TARGET == LCD_PICO_DISPLAY_2) || (LCD_TARGET == LCD_WAVESHARE28)
    .max_buffer_size = sizeof(compress_buf),
#endif

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
#if LCD_TARGET == LCD_PICO_DISPLAY
for (int i = 0; i < HEIGHT * WIDTH; i++) {
        framebuffer[i] = TFT_GREEN;
    }
    backlight_init(BL_GPIO);
    mipi_dbi_spi_init(&dbi);

    mipi_dbi_command(&dbi, MIPI_DCS_SOFT_RESET);

    sleep_ms(150);

    mipi_dbi_command(&dbi, MIPI_DCS_SET_ADDRESS_MODE, 0x70);
    mipi_dbi_command(&dbi, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FORMAT_16BIT);

    mipi_dbi_command(&dbi, MIPI_DCS_ENTER_INVERT_MODE);
    mipi_dbi_command(&dbi, MIPI_DCS_EXIT_SLEEP_MODE);
    mipi_dbi_command(&dbi, MIPI_DCS_SET_DISPLAY_ON);

    sleep_ms(100);

    // Clear display
    mipi_dbi_update16(&dbi, START_X, START_Y, WIDTH, HEIGHT, framebuffer, WIDTH * HEIGHT * 2);
    backlight_set(BL_DEF_LEVEL);
#elif LCD_TARGET == LCD_PICO_DISPLAY_2
for (int i = 0; i < HEIGHT * WIDTH; i++) {
        framebuffer[i] = TFT_GREEN;
    }
    backlight_init(BL_GPIO);
    mipi_dbi_spi_init(&dbi);

    mipi_dbi_command(&dbi, MIPI_DCS_SOFT_RESET);

    sleep_ms(150);

    mipi_dbi_command(&dbi, MIPI_DCS_SET_ADDRESS_MODE, 0x70);
    mipi_dbi_command(&dbi, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FORMAT_16BIT);

    mipi_dbi_command(&dbi, MIPI_DCS_ENTER_INVERT_MODE);
    mipi_dbi_command(&dbi, MIPI_DCS_EXIT_SLEEP_MODE);
    mipi_dbi_command(&dbi, MIPI_DCS_SET_DISPLAY_ON);

    sleep_ms(100);

    // Clear display
    mipi_dbi_update16(&dbi, START_X, START_Y, WIDTH, HEIGHT, framebuffer, WIDTH * HEIGHT * 2);
    backlight_set(BL_DEF_LEVEL);
#elif LCD_TARGET == LCD_WAVESHARE28
    backlight_init(BL_GPIO);
    mipi_dbi_spi_init(&dbi);

    if (LCD_RESET >= 0)
        mipi_dbi_hw_reset(LCD_RESET);

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
    mipi_dbi_command(&dbi, MIPI_DCS_SET_GAMMA_CURVE, 0x01);
    mipi_dbi_command(&dbi, ILI9341_PGAMCTRL,
                     0x1f, 0x1a, 0x18, 0x0a, 0x0f, 0x06, 0x45, 0x87,
                     0x32, 0x0a, 0x07, 0x02, 0x07, 0x05, 0x00);
    mipi_dbi_command(&dbi, ILI9341_NGAMCTRL,
                     0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3a, 0x78,
                     0x4d, 0x05, 0x18, 0x0d, 0x38, 0x3a, 0x1f);

    /* DDRAM */
    mipi_dbi_command(&dbi, ILI9341_ETMOD, 0x07);

    /* Display */
    mipi_dbi_command(&dbi, ILI9341_DISCTRL, 0x0a, 0x82, 0x27, 0x00);
    mipi_dbi_command(&dbi, MIPI_DCS_EXIT_SLEEP_MODE);
    sleep_ms(100);

    uint16_t rotation = 180;
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
    sleep_ms(100);

    // Clear display
    mipi_dbi_update16(&dbi, 0, 0, WIDTH, HEIGHT, framebuffer, WIDTH * HEIGHT * 2);
#endif
}

static void pwm_gpio_init(uint gpio, uint16_t val)
{
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_output_polarity(&cfg, true, true);
    pwm_set_wrap(pwm_gpio_to_slice_num(gpio), 65535);
    pwm_init(pwm_gpio_to_slice_num(gpio), &cfg, true);
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    pwm_set_gpio_level(gpio, val);
}

static void turn_off_rgb_led(void)
{
#if defined(LCD_LED_R)
    pwm_gpio_init(LCD_LED_R, 0);
#endif
#if defined(LCD_LED_G)
    pwm_gpio_init(LCD_LED_G, 0);
#endif
#if defined(LCD_LED_B)
    pwm_gpio_init(LCD_LED_B, 0);
#endif
}

int main(void)
{
    board_init();

    if (USE_WATCHDOG && watchdog_caused_reboot()) {
        LOG("Rebooted by Watchdog!\n");
        panic_reboot_blink_time = 1;
    }

    if (LED_ACTION)
        board_led_write(true);

#if LCD_TARGET == LCD_WAVESHARE28
    // Pull ADS7846 CE1 high
    gpio_set_function(XPT2046_CS, GPIO_FUNC_SIO);
    gpio_set_dir(XPT2046_CS, GPIO_OUT);
    gpio_put(XPT2046_CS, 1);
#endif

    init_display();

    gud_driver_setup(&display, framebuffer, compress_buf);

    tusb_init();

    LOG("\n\n%s: CFG_TUSB_DEBUG=%d\n", __func__, CFG_TUSB_DEBUG);

    turn_off_rgb_led();

    if (USE_WATCHDOG)
        watchdog_enable(5000, 0); // pause_on_debug=0 so it can reset panics.

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
