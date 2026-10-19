#include "mosaico_power.h"

#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static bool power_enabled;

esp_err_t mosaico_power_enable(void) {
    if(power_enabled) return ESP_OK;

    esp_err_t err=gpio_set_level(GPIO_NUM_60,1);
    if(err!=ESP_OK) return err;
    const gpio_config_t output={.pin_bit_mask=1ULL<<60,.mode=GPIO_MODE_OUTPUT};
    err=gpio_config(&output);
    if(err!=ESP_OK) return err;
    err=gpio_hold_dis(GPIO_NUM_60);
    if(err!=ESP_OK) return err;

    const ledc_timer_config_t timer={.speed_mode=LEDC_LOW_SPEED_MODE,.timer_num=LEDC_TIMER_1,
        .duty_resolution=LEDC_TIMER_8_BIT,.freq_hz=100000,.clk_cfg=LEDC_AUTO_CLK};
    err=ledc_timer_config(&timer);
    if(err!=ESP_OK) return err;
    const ledc_channel_config_t channel={.gpio_num=GPIO_NUM_60,.speed_mode=LEDC_LOW_SPEED_MODE,
        .channel=LEDC_CHANNEL_1,.timer_sel=LEDC_TIMER_1,.duty=255};
    err=ledc_channel_config(&channel);
    if(err!=ESP_OK) return err;

    esp_err_t installed=ledc_fade_func_install(0);
    err=installed==ESP_ERR_INVALID_STATE ? ESP_OK : installed;
    if(err==ESP_OK) err=ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1,0,50);
    if(err==ESP_OK) err=ledc_fade_start(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1,LEDC_FADE_WAIT_DONE);
    if(installed==ESP_OK) ledc_fade_func_uninstall();

    int level=err==ESP_OK ? 0 : 1;
    ledc_stop(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1,level);
    esp_err_t restore=gpio_set_level(GPIO_NUM_60,level);
    if(restore==ESP_OK) restore=gpio_config(&output);
    if(err==ESP_OK) err=restore;
    if(err==ESP_OK) {
        power_enabled=true;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return err;
}
