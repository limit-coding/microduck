#include "mosaico_imu.h"
#include "bmi270.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mosaico_power.h"

/* User-confirmed ESP-Mosaico V1.2: SDA56/SCL3, IMU address 0x69.
 * Board mapping and GPIO60 active-low, 50 ms rail ramp follow esp-mosaico-bsp. */
static portMUX_TYPE guard=portMUX_INITIALIZER_UNLOCKED;
static mosaico_imu_snapshot_t latest={.error=ESP_ERR_INVALID_STATE};

void mosaico_imu_get(mosaico_imu_snapshot_t *out) {
    portENTER_CRITICAL(&guard);
    *out=latest;
    portEXIT_CRITICAL(&guard);
}
static void publish(const mosaico_imu_snapshot_t *s) {
    portENTER_CRITICAL(&guard);
    latest=*s;
    portEXIT_CRITICAL(&guard);
}
static esp_err_t read_regs(bmi270_handle_t *imu, uint8_t reg, uint8_t *data, size_t size) {
    return i2c_master_transmit_receive(imu->i2c_handle,&reg,1,data,size,50);
}
static esp_err_t init_imu(i2c_master_bus_handle_t bus, bmi270_handle_t **imu, uint8_t *chip_id) {
    *chip_id=0;
    esp_err_t err=i2c_master_probe(bus,BMI270_I2C_ADDRESS_H,50);
    if(err!=ESP_OK) return err;
    const bmi270_driver_config_t driver={.addr=BMI270_I2C_ADDRESS_H,.interface=BMI270_USE_I2C,.i2c_bus=bus};
    err=bmi270_create(&driver,imu); /* Includes reset, Bosch config upload and internal-status check. */
    if(err!=ESP_OK) return err;
    err=bmi270_get_chip_id(*imu,chip_id);
    if(err!=ESP_OK) return err;
    const bmi270_config_t config={.acce_odr=BMI270_ACC_ODR_100_HZ,.acce_range=BMI270_ACC_RANGE_4_G,
        .gyro_odr=BMI270_GYR_ODR_100_HZ,.gyro_range=BMI270_GYR_RANGE_2000_DPS};
    err=bmi270_start(*imu,&config);
    if(err!=ESP_OK) return err;
    /* Standard sensor bandwidth; raw readings and policy gyro have no added low-pass. */
    err=bmi270_set_acce_filter_perf(*imu,BMI270_PERFORMANCE_OPTIMIZED);
    if(err!=ESP_OK) return err;
    err=bmi270_set_acce_bwp(*imu,BMI270_ACC_BWP_NORM_AVG4);
    if(err!=ESP_OK) return err;
    err=bmi270_set_gyro_filter_perf(*imu,BMI270_PERFORMANCE_OPTIMIZED);
    if(err!=ESP_OK) return err;
    err=bmi270_set_gyro_noise_perf(*imu,BMI270_PERFORMANCE_OPTIMIZED);
    if(err!=ESP_OK) return err;
    err=bmi270_set_gyro_bwp(*imu,BMI270_GYR_BWP_NORM);
    if(err==ESP_OK) vTaskDelay(pdMS_TO_TICKS(100));
    return err;
}
static void imu_task(void *arg) {
    (void)arg;
    mosaico_imu_snapshot_t s={.error=ESP_ERR_INVALID_STATE};
    i2c_master_bus_handle_t bus=NULL;
    bmi270_handle_t *imu=NULL;
    const i2c_master_bus_config_t config={.i2c_port=I2C_NUM_0,.sda_io_num=GPIO_NUM_56,.scl_io_num=GPIO_NUM_3,
        .clk_source=I2C_CLK_SRC_DEFAULT,.glitch_ignore_cnt=7,.flags.enable_internal_pullup=true};
    bool power_ready=false;
    mosaico_estimator_t estimator={0};
    int64_t last_progress=0;
    for(;;) {
        if(!imu) {
            s.ready=false;s.estimate_valid=false;estimator.initialized=false;
            if(!power_ready) {
                s.error=mosaico_power_enable();power_ready=s.error==ESP_OK;
                if(!power_ready) {s.errors++;publish(&s);vTaskDelay(pdMS_TO_TICKS(2000));continue;}
            }
            s.error=bus ? ESP_OK : i2c_new_master_bus(&config,&bus);
            if(s.error==ESP_OK) s.error=init_imu(bus,&imu,&s.chip_id);
            if(s.error!=ESP_OK) {
                if(imu) {bmi270_delete(imu);imu=NULL;}
                s.errors++;publish(&s);
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            s.ready=true;last_progress=esp_timer_get_time();publish(&s);
        }
        uint8_t status=0,data[15];
        s.error=read_regs(imu,0x03,&status,1);
        /* Both data-ready flags must be set; don't count repeated register reads as samples. */
        if(s.error==ESP_OK && (status&0xc0)==0xc0) {
            s.error=read_regs(imu,0x0c,data,sizeof(data));
            if(s.error==ESP_OK) {
                mosaico_imu_decode(data,s.acc,s.gyro,&s.sensor_time);
                int64_t sampled=esp_timer_get_time();
                float dt=estimator.initialized ? (sampled-s.sample_us)*1e-6f : 0.01f;
                s.estimate_valid=mosaico_estimate(&estimator,s.acc,s.gyro,dt,s.acc_g,s.gyro_rad,s.gravity);
                s.sample_us=sampled;last_progress=sampled;s.samples++;
                publish(&s);
            }
        }
        if(s.error==ESP_OK && esp_timer_get_time()-last_progress>1000000) s.error=ESP_ERR_TIMEOUT;
        if(s.error!=ESP_OK) {
            s.ready=false;s.estimate_valid=false;estimator.initialized=false;s.errors++;publish(&s);
            bmi270_delete(imu);imu=NULL;
            vTaskDelay(pdMS_TO_TICKS(2000));
        } else vTaskDelay(pdMS_TO_TICKS(5));
    }
}
void mosaico_imu_start(void) {
    if(xTaskCreate(imu_task,"mosaico_imu",4096,NULL,3,NULL)!=pdPASS) {
        mosaico_imu_snapshot_t failed={.error=ESP_ERR_NO_MEM,.errors=1};
        publish(&failed);
    }
}
