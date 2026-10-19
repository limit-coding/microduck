#include "duck_display.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_co5300.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "duck_eye_asset.h"

#define LCD_HOST SPI2_HOST
#define LCD_PIN_DATA3 9
#define LCD_PIN_DATA2 35
#define LCD_PIN_DATA0 36
/* ESP-Mosaico V1.2 swaps the V1.0 LCD reset and clock pins. */
#define LCD_PIN_RST 44
#define LCD_PIN_CLK 42
#define LCD_PIN_CS 50
#define LCD_PIN_DATA1 51
#define LCD_STRIP_ROWS 20

static const char *TAG="duck_display";
static esp_lcd_panel_handle_t panel;
static esp_lcd_panel_io_handle_t panel_io;

/* Mosaico's 480x480 glass needs the supplier sequence from the official BSP.
 * The generic CO5300 defaults describe a smaller, offset window and do not
 * program the panel power page used by this module.  TE is omitted because
 * this application transfers one frame and never refreshes it. */
static const co5300_lcd_init_cmd_t mosaico_init[] = {
    {0x11, NULL, 0, 600},
    {0xFE, (uint8_t[]){0x20}, 1, 0},
    {0x19, (uint8_t[]){0x10}, 1, 0},
    {0x1C, (uint8_t[]){0xA0}, 1, 0},
    {0xFE, (uint8_t[]){0x00}, 1, 0},
    {0xC4, (uint8_t[]){0x80}, 1, 0},
    {0x3A, (uint8_t[]){0x55}, 1, 0},
    {0x53, (uint8_t[]){0x20}, 1, 0},
    {0x51, (uint8_t[]){0xFF}, 1, 0},
    {0x63, (uint8_t[]){0xFF}, 1, 0},
    {0x2A, (uint8_t[]){0x00,0x00,0x01,0xDF}, 4, 0},
    {0x2B, (uint8_t[]){0x00,0x00,0x01,0xDF}, 4, 0},
    {0x29, NULL, 0, 600},
};

static bool IRAM_ATTR color_done(esp_lcd_panel_io_handle_t io,
                                 esp_lcd_panel_io_event_data_t *event,
                                 void *user_ctx) {
    (void)io;
    (void)event;
    BaseType_t wake=pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)user_ctx,&wake);
    return wake==pdTRUE;
}

static inline uint16_t wire_rgb565(uint16_t value) {
    return (uint16_t)((value<<8)|(value>>8));
}

static esp_err_t draw_eye_once(SemaphoreHandle_t done) {
    const size_t strip_pixels=DUCK_EYE_WIDTH*LCD_STRIP_ROWS;
    uint16_t *pixels=heap_caps_malloc(strip_pixels*sizeof(uint16_t),MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL);
    if(!pixels) return ESP_ERR_NO_MEM;

    size_t run_index=0;
    uint32_t run_left=0;
    uint16_t run_color=0;
    esp_err_t err=ESP_OK;
    for(int y=0;y<DUCK_EYE_HEIGHT && err==ESP_OK;y+=LCD_STRIP_ROWS) {
        const int rows=(y+LCD_STRIP_ROWS<=DUCK_EYE_HEIGHT)?LCD_STRIP_ROWS:DUCK_EYE_HEIGHT-y;
        size_t needed=(size_t)DUCK_EYE_WIDTH*rows;
        size_t written=0;
        while(written<needed) {
            if(run_left==0) {
                if(run_index>=duck_eye_run_count) {err=ESP_ERR_INVALID_SIZE;break;}
                run_left=duck_eye_runs[run_index].count;
                run_color=wire_rgb565(duck_eye_runs[run_index].rgb565);
                run_index++;
            }
            size_t count=needed-written;
            if(count>run_left) count=run_left;
            for(size_t i=0;i<count;i++) pixels[written+i]=run_color;
            written+=count;
            run_left-=count;
        }
        if(err!=ESP_OK) break;
        err=esp_lcd_panel_draw_bitmap(panel,0,y,DUCK_EYE_WIDTH,y+rows,pixels);
        if(err==ESP_OK && xSemaphoreTake(done,pdMS_TO_TICKS(1000))!=pdTRUE) err=ESP_ERR_TIMEOUT;
    }
    if(err==ESP_OK && (run_left!=0 || run_index!=duck_eye_run_count)) err=ESP_ERR_INVALID_SIZE;
    free(pixels);
    return err;
}

esp_err_t duck_display_show_static(void) {
    SemaphoreHandle_t done=xSemaphoreCreateBinary();
    if(!done) return ESP_ERR_NO_MEM;

    const spi_bus_config_t bus=CO5300_PANEL_BUS_QSPI_CONFIG(
        LCD_PIN_CLK,LCD_PIN_DATA0,LCD_PIN_DATA1,LCD_PIN_DATA2,LCD_PIN_DATA3,
        DUCK_EYE_WIDTH*LCD_STRIP_ROWS*sizeof(uint16_t));
    esp_err_t err=spi_bus_initialize(LCD_HOST,&bus,SPI_DMA_CH_AUTO);
    if(err!=ESP_OK) goto exit;

    const gpio_num_t qspi_pins[]={LCD_PIN_CLK,LCD_PIN_DATA0,LCD_PIN_DATA1,LCD_PIN_DATA2,LCD_PIN_DATA3};
    for(size_t i=0;i<sizeof(qspi_pins)/sizeof(qspi_pins[0]);i++) {
        err=gpio_set_drive_capability(qspi_pins[i],GPIO_DRIVE_CAP_0);
        if(err!=ESP_OK) goto exit;
    }

    esp_lcd_panel_io_spi_config_t io=CO5300_PANEL_IO_QSPI_CONFIG(LCD_PIN_CS,color_done,done);
    io.trans_queue_depth=1;
    err=esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,&io,&panel_io);
    if(err!=ESP_OK) goto exit;

    const co5300_vendor_config_t vendor={
        .init_cmds=mosaico_init,
        .init_cmds_size=sizeof(mosaico_init)/sizeof(mosaico_init[0]),
        .flags={.use_qspi_interface=1},
    };
    const esp_lcd_panel_dev_config_t config={
        .reset_gpio_num=LCD_PIN_RST,
        .rgb_ele_order=LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel=16,
        .vendor_config=(void *)&vendor,
    };
    err=esp_lcd_new_panel_co5300(panel_io,&config,&panel);
    if(err==ESP_OK) err=esp_lcd_panel_reset(panel);
    if(err==ESP_OK) err=esp_lcd_panel_init(panel);
    if(err==ESP_OK) err=esp_lcd_panel_disp_on_off(panel,true);
    if(err==ESP_OK) err=esp_lcd_panel_co5300_set_brightness(panel,100);
    /* Native orientation: source eye stays on the physical right/camera side. */
    if(err==ESP_OK) err=draw_eye_once(done);
    if(err==ESP_OK) ESP_LOGI(TAG,"V1.2 static 480x480 eye displayed (SCL=%d RST=%d); no refresh task started",
                             LCD_PIN_CLK,LCD_PIN_RST);

exit:
    vSemaphoreDelete(done);
    return err;
}
