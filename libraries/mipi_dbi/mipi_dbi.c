// SPDX-License-Identifier: MIT

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "mipi_dbi.h"

/* コンパイルの選択 */
#define GUD_DEBUG 0
#define USE_DMA 1
#define DBI_DEBUG 0
#define DBI_TIME_SPI 1

// LOG3
#include <stdio.h>
#if GUD_DEBUG == 1
#define LOG     printf
#else
#define LOG
#endif
#if GUD_DEBUG == 2
#define LOG2    printf
#else
#define LOG2
#endif
#if GUD_DEBUG >= 3
#define LOG3    printf
#else
#define LOG3
#endif

#if DBI_DEBUG>=1
#define DBI_LOG printf
#else
#define DBI_LOG
#endif
#if DBI_DEBUG>=2
#define DBI_LOG2 printf
#else
#define DBI_LOG2
#endif
static uint dma_channels[2] = { ~0, ~0 };

/*
 * Many controllers have a max speed of 10MHz, but can be pushed way beyond
 * that. Increase reliability by running pixel data at max speed and the rest
 * at 10MHz, preventing transfer glitches from messing up the init settings.
 */

void mipi_dbi_command_buf(const struct mipi_dbi *dbi, uint8_t cmd, const uint8_t *data, size_t len)
{
    DBI_LOG2("DCS:%s, %02x",__func__, cmd);
    for (uint8_t i = 0; i < (len > 64 ? 64 : len); i++)
        DBI_LOG2(" %02x", data[i]);
    DBI_LOG2("\n");

    spi_set_format(dbi->spi, 8, dbi->cpol, dbi->cpha, SPI_MSB_FIRST);

    if (dbi->baudrate > 10 * 1000 * 1000)
        spi_set_baudrate(dbi->spi, 10 * 1000 * 1000);

    gpio_put(dbi->cs, 0);

    gpio_put(dbi->dc, 0);
    spi_write_blocking(dbi->spi, &cmd, 1);

    if (len) {
        gpio_put(dbi->dc, 1);
        spi_write_blocking(dbi->spi, data, len);
    }

    gpio_put(dbi->cs, 1);
}

void mipi_dbi_set_window(const struct mipi_dbi *dbi,
                         uint16_t x, uint16_t y,
                         uint16_t width, uint16_t height)
{
    uint16_t xe = x + width - 1;
    uint16_t ye = y + height - 1;

    mipi_dbi_command(dbi, MIPI_DCS_SET_COLUMN_ADDRESS,
                     x >> 8, x & 0xff, xe >> 8, xe & 0xff);
    mipi_dbi_command(dbi, MIPI_DCS_SET_PAGE_ADDRESS,
                     y >> 8, y & 0xff, ye >> 8, ye & 0xff);
}

static void mipi_dbi_update16_dma(const struct mipi_dbi *dbi, uint16_t x, uint16_t y,
                                  uint16_t width, uint16_t height, void *buf, size_t len)
{
    uint64_t start ;
    if (DBI_TIME_SPI)
        start = time_us_64();
    uint idx = spi_get_index(dbi->spi);
    uint dma_channel = dma_channels[idx];
    LOG3("%s, x: %03d, y: %03d, w: %03d, h:%03d, len: % 4d ",__func__,x,y,width,height,len);

    if (dma_channel == ~0) {
        dma_channel = dma_claim_unused_channel(true);
        dma_channel_config config = dma_channel_get_default_config(dma_channel);
        channel_config_set_transfer_data_size(&config, DMA_SIZE_16);
        channel_config_set_dreq(&config, idx ? DREQ_SPI1_TX : DREQ_SPI0_TX);
        dma_channel_configure(dma_channel, &config, &spi_get_hw(dbi->spi)->dr,
            buf, width * height, false);

//    dma_channel_set_read_addr(dma_channel, framebuffer, false);
//    dma_channel_set_write_addr(dma_channel, &spi_get_hw(dbi.spi)->dr, false);
//    dma_channel_set_trans_count(dma_channel, transfer_count, false);
//    dma_channel_set_config(dma_channel, &config, false);

        dma_channels[idx] = dma_channel;
    }

    dma_channel_wait_for_finish_blocking(dma_channel);

    mipi_dbi_set_window(dbi, x, y, width, height);

    gpio_put(dbi->cs, 0);

    gpio_put(dbi->dc, 0);
    uint8_t cmd = MIPI_DCS_WRITE_MEMORY_START;
    spi_write_blocking(dbi->spi, &cmd, 1);

    gpio_put(dbi->dc, 1);

    spi_set_format(dbi->spi, 16, dbi->cpol, dbi->cpha, SPI_MSB_FIRST);
    spi_set_baudrate(dbi->spi, dbi->baudrate);

    dma_channel_set_read_addr(dma_channel, buf, false);
    dma_channel_set_trans_count(dma_channel, width * height, true);

    if (DBI_TIME_SPI) {
        dma_channel_wait_for_finish_blocking(dma_channel);
        LOG3("%s , %llu ms\n", __func__ , (time_us_64() - start)/1000);
    }
}

void mipi_dbi_update16(const struct mipi_dbi *dbi, uint16_t x, uint16_t y,
                       uint16_t width, uint16_t height, void *buf, size_t len)
{
    static bool toggle=true;
    //toggle=!toggle;
    if (USE_DMA && toggle) {
        mipi_dbi_update16_dma(dbi, x, y, width, height, buf, len);
        return;
    }
    uint64_t start ;
    if (DBI_TIME_SPI)
        start = time_us_64();

    mipi_dbi_set_window(dbi, x, y, width, height);

    gpio_put(dbi->cs, 0);    gpio_put(dbi->dc, 0);
    uint8_t cmd = MIPI_DCS_WRITE_MEMORY_START;
    spi_write_blocking(dbi->spi, &cmd, 1);

    gpio_put(dbi->dc, 1);

    spi_set_format(dbi->spi, 16, dbi->cpol, dbi->cpha, SPI_MSB_FIRST);
    spi_set_baudrate(dbi->spi, dbi->baudrate);

    spi_write16_blocking(dbi->spi, buf, len / 2);

    LOG3("%s, x: %03d, y: %03d, w: %03d, h:%03d, len: % 4d ",__func__,x,y,width,height,len);

    if (DBI_TIME_SPI)
        LOG3("%s , %llu ms\n", __func__ , (time_us_64() - start)/1000);

    gpio_put(dbi->cs, 1);
}
//#define MAX_PACKET_SIZE 1024*3 // Adjust this value as needed based on spi buffer size
void mipi_dbi_update16_24(const struct mipi_dbi *dbi, uint16_t x, uint16_t y,
                       uint16_t width, uint16_t height, void *buf, size_t len)
{
    uint8_t *Byte,*buf2;
    uint64_t start;
    static bool toggle = true;
    //toggle = !toggle;
    LOG3("%s, ",__func__);
    LOG2("%s, ",__func__);

    if (DBI_TIME_SPI)
        start = time_us_64();

    int MAX_PACKET_SIZE;
    int line=64 ;
    MAX_PACKET_SIZE = width*line*3;
    Byte = malloc(MAX_PACKET_SIZE);
    if (Byte==0){
        LOG3("%s: malloc error, 0x%08x\n\r",__func__,Byte);
        return;
    }
    LOG3(", x: %03d, y: %03d, w: %03d, h:%03d, len: %4dk ",x,y,width,height,len/1000);
    LOG2(", malloc 0x%08x\n\r",Byte);
    mipi_dbi_set_window(dbi, x, y, width, height);

    gpio_put(dbi->cs, 0);
    gpio_put(dbi->dc, 0);
    uint8_t cmd = MIPI_DCS_WRITE_MEMORY_START;
    spi_write_blocking(dbi->spi, &cmd, 1);
    gpio_put(dbi->dc, 1);

    spi_set_format(dbi->spi, 8, dbi->cpol, dbi->cpha, SPI_MSB_FIRST);
    spi_set_baudrate(dbi->spi, dbi->baudrate);
    #if 1
        uint idx = spi_get_index(dbi->spi);
        uint dma_channel = dma_channels[idx];
        if (USE_DMA && toggle) {
            if (dma_channel == ~0) {
                dma_channel = dma_claim_unused_channel(true);
                dma_channel_config config = dma_channel_get_default_config(dma_channel);
                channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
                channel_config_set_dreq(&config, idx ? DREQ_SPI1_TX : DREQ_SPI0_TX);
                dma_channel_configure(dma_channel, &config, &spi_get_hw(dbi->spi)->dr,
                    buf, width * 3, false);
                dma_channels[idx] = dma_channel;
            }
        }    
        uint16_t* pos=buf;
        int length=width*height;
        buf2=Byte;
        //LOG3("%s: Loop count %d.%d , MAX_PACKET_SIZE:%d\n\r",__func__,
        //    (len>>1)/(MAX_PACKET_SIZE/3) , 
        //    (len>>1)%(MAX_PACKET_SIZE/3),MAX_PACKET_SIZE/3);
        int count=0;
        for (size_t dots = 0 ; dots < length ; dots += MAX_PACKET_SIZE/3 ) {
            size_t chunkSize = (length - dots);
            if (chunkSize > MAX_PACKET_SIZE/3 ) {
                chunkSize = MAX_PACKET_SIZE/3 ;
            }
            for (size_t j = 0; j < chunkSize; j++) {
                buf2[j*3+2] = (( *pos & 0xf800 )>>11)<<3;//Red
                buf2[j*3+1] = (( *pos & 0x07e0 )>>5 )<<2;//Green
                buf2[j*3  ] =  ( *pos & 0x001f )<<3     ;//Blue
                pos++;
            }
            if (USE_DMA && toggle) {
                dma_channel_set_read_addr(dma_channel, Byte, false);
                dma_channel_set_trans_count(dma_channel, chunkSize *3, true);
                dma_channel_wait_for_finish_blocking(dma_channel);
            }else{
                spi_write_blocking(dbi->spi, Byte, chunkSize*3);
            }
            count++;
    
        }
        gpio_put(dbi->cs, 1);

        LOG3("loop : %d,",count);
    #else
        uint idx = spi_get_index(dbi->spi);
        uint dma_channel = dma_channels[idx];
        if (USE_DMA && toggle) {
            if (dma_channel == ~0) {
                dma_channel = dma_claim_unused_channel(true);
                dma_channel_config config = dma_channel_get_default_config(dma_channel);
                channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
                channel_config_set_dreq(&config, idx ? DREQ_SPI1_TX : DREQ_SPI0_TX);
                dma_channel_configure(dma_channel, &config, &spi_get_hw(dbi->spi)->dr,
                    buf, width * 3, false);
                dma_channels[idx] = dma_channel;
            }
        }    
        
        uint16_t* pos=buf;
        buf2=Byte;
        for(int i=0;i<height;i++){
            for(int j=0; j<width;j++){
                buf2[j*3+2] = (( *pos & 0xf800 )>>11)<<3;//Red
                buf2[j*3+1] = (( *pos & 0x07e0 )>>5 )<<2;//Green
                buf2[j*3  ] =  ( *pos & 0x001f )<<3     ;//Blue
                pos++;
            }
            if (USE_DMA && toggle) {
                dma_channel_set_read_addr(dma_channel, Byte, false);
                dma_channel_set_trans_count(dma_channel, width *3, true);
                dma_channel_wait_for_finish_blocking(dma_channel);
            }else{
                spi_write_blocking(dbi->spi, Byte, width*3);
            }
        }

        gpio_put(dbi->cs, 1);
    #endif
    free(Byte);

    if (DBI_TIME_SPI)
        if(USE_DMA)
            if (toggle) {
                LOG3(" write=%llu ms DMA\n",(time_us_64() - start)/1000);
            }else{
                LOG3(" write=%llu ms\n", (time_us_64() - start)/1000);
            }
        else
            LOG3(" write=%llu ms\n",(time_us_64() - start)/1000);
}
static void mipi_dbi_update24_dma(const struct mipi_dbi *dbi, uint16_t x, uint16_t y,
                                  uint16_t width, uint16_t height, void *buf, size_t len)
{
    //LOG3("%s, x: %d, y: %d, w: %d, h:%d, len: %d\n\r",__func__,x,y,width,height,len);
    uint64_t start ;
    if (DBI_TIME_SPI) {
        start = time_us_64();
    }
    uint idx = spi_get_index(dbi->spi);
    uint dma_channel = dma_channels[idx];

    if (dma_channel == ~0) {
        dma_channel = dma_claim_unused_channel(true);
        dma_channel_config config = dma_channel_get_default_config(dma_channel);
        channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
        channel_config_set_dreq(&config, idx ? DREQ_SPI1_TX : DREQ_SPI0_TX);
        dma_channel_configure(dma_channel, &config, &spi_get_hw(dbi->spi)->dr,
            buf, width * height*3, false);

//    dma_channel_set_read_addr(dma_channel, framebuffer, false);
//    dma_channel_set_write_addr(dma_channel, &spi_get_hw(dbi.spi)->dr, false);
//    dma_channel_set_trans_count(dma_channel, transfer_count, false);
//    dma_channel_set_config(dma_channel, &config, false);

        dma_channels[idx] = dma_channel;
    }

    dma_channel_wait_for_finish_blocking(dma_channel);

    mipi_dbi_set_window(dbi, x, y, width, height);

    gpio_put(dbi->cs, 0);

    gpio_put(dbi->dc, 0);
    uint8_t cmd = MIPI_DCS_WRITE_MEMORY_START;
    spi_write_blocking(dbi->spi, &cmd, 1);

    gpio_put(dbi->dc, 1);

    spi_set_format(dbi->spi, 8, dbi->cpol, dbi->cpha, SPI_MSB_FIRST);
    spi_set_baudrate(dbi->spi, dbi->baudrate);


    dma_channel_set_read_addr(dma_channel, buf, false);
    dma_channel_set_trans_count(dma_channel, width * height*3, true);

    if (DBI_TIME_SPI) {
        dma_channel_wait_for_finish_blocking(dma_channel);
        LOG3("%s , %llu ms\n", __func__ , (time_us_64() - start)/1000);
    }
}
void mipi_dbi_update24(const struct mipi_dbi *dbi, uint16_t x, uint16_t y,
                       uint16_t width, uint16_t height, void *buf, size_t len)
{    
    int w=len/(height*width);
    static bool toggle=false;
    if(0){
    }
    toggle=!toggle;
    if (USE_DMA && toggle && w==3) {
        //LOG3("USE_DMA: %s\n\r",USE_DMA);
        //RGB->BGR
        #if 0
        uint8_t temp,*pos=buf;
        for(int i=0;i<height;i++){
            for(int j=0; j<width;j++){
                temp    = *pos;
                *pos    = *(pos+2);
                *(pos+2)= temp;
                pos+=3;
            }
        }
        #endif
        mipi_dbi_update24_dma(dbi, x, y, width, height, buf, len);
        return;
    }
    uint64_t start ;
    if (DBI_TIME_SPI)
        start = time_us_64();

    mipi_dbi_set_window(dbi, x, y, width, height);

    gpio_put(dbi->cs, 0);

    gpio_put(dbi->dc, 0);
    uint8_t cmd = MIPI_DCS_WRITE_MEMORY_START;
    spi_write_blocking(dbi->spi, &cmd, 1);

    gpio_put(dbi->dc, 1);

    spi_set_format(dbi->spi, 8, dbi->cpol, dbi->cpha, SPI_MSB_FIRST);
    spi_set_baudrate(dbi->spi, dbi->baudrate);

    
    {
        //int w=len/(height*width);
        if( w==2){
            uint8_t buf2[width*3];
            uint16_t* pos=buf;
            for(int i=0;i<height;i++){
                for(int j=0; j<width;j++){
                    buf2[j*3]   = (( *pos & 0xf800 )>>11)<<2;
                    buf2[j*3+1] = (( *pos & 0x07e0 )>>5)<<2;
                    buf2[j*3+2] = ( *pos & 0x001f )<<2;
                    pos++;
                }
                spi_write_blocking(dbi->spi, buf2, width*3);
            }
        }else if(w==3){
            uint8_t temp,*pos=buf;
            #if 0
            for(int i=0;i<height;i++){
                for(int j=0; j<width;j++){
                    temp    = *pos;
                    *pos    = *(pos+2);
                    *(pos+2)= temp;
                    pos+=3;
                }
            }
            #endif
            spi_write_blocking(dbi->spi, buf, len);
            //LOG3("%s, w:%d, x: %d, y: %d, w: %d, h:%d, len: %d\n\r",__func__,w,x,y,width,height,len);
        }else{
            LOG3("%s, w:%d, x: %d, y: %d, w: %d, h:%d, len: %d\n\r",__func__,w,x,y,width,height,len);
        }
    }

    if (DBI_TIME_SPI)
        LOG3("%s , %llu ms\n", __func__ , (time_us_64() - start)/1000);

    gpio_put(dbi->cs, 1);
}

void mipi_dbi_update_wait(const struct mipi_dbi *dbi)
{
    if (USE_DMA) {
        uint idx = spi_get_index(dbi->spi);
        uint dma_channel = dma_channels[idx];

        if (dma_channel != ~0) {
            if (dma_channel_is_busy(dma_channel))
                DBI_LOG("Waiting for DMA to finish\n");
            dma_channel_wait_for_finish_blocking(dma_channel);
        }
    }
}

void mipi_dbi_spi_init(const struct mipi_dbi *dbi)
{
    spi_init(dbi->spi, dbi->baudrate);

    gpio_set_function(dbi->sck,  GPIO_FUNC_SPI);
    gpio_set_function(dbi->mosi, GPIO_FUNC_SPI);

    gpio_set_function(dbi->cs, GPIO_FUNC_SIO);
    gpio_set_dir(dbi->cs, GPIO_OUT);

    gpio_set_function(dbi->dc, GPIO_FUNC_SIO);
    gpio_set_dir(dbi->dc, GPIO_OUT);
}

void mipi_dbi_hw_reset(uint gpio)
{
    // init pin if not already done by the caller
    if (gpio_get_function(gpio) != GPIO_FUNC_SIO) {
        gpio_set_function(gpio, GPIO_FUNC_SIO);
        gpio_set_dir(gpio, GPIO_OUT);
    }
    gpio_put(gpio, 0);
    sleep_us(20);
    gpio_put(gpio, 1);
    sleep_ms(120);
}
