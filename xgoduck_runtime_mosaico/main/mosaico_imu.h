#pragma once
#include <stdbool.h>
#include <stdint.h>

#define MOSAICO_IMU_MAX_AGE_US 100000
#ifdef DUCK_HOST_TEST
#define MOSAICO_API __declspec(dllexport)
#else
#define MOSAICO_API
#endif

typedef struct { bool initialized; float gravity[3]; } mosaico_estimator_t;

typedef struct {
    bool ready, estimate_valid;
    uint8_t chip_id;
    int error;
    int16_t acc[3], gyro[3];
    float acc_g[3], gyro_rad[3], gravity[3];
    uint32_t samples, errors, sensor_time;
    int64_t sample_us;
} mosaico_imu_snapshot_t;

void mosaico_imu_start(void);
void mosaico_imu_get(mosaico_imu_snapshot_t *out);
#ifdef DUCK_HOST_TEST
__declspec(dllexport)
#endif
void mosaico_imu_decode(const uint8_t data[15], int16_t acc[3], int16_t gyro[3], uint32_t *sensor_time);

MOSAICO_API bool mosaico_estimate(mosaico_estimator_t *filter, const int16_t acc[3], const int16_t gyro[3], float dt,
                                float acc_g[3], float gyro_rad[3], float gravity[3]);
MOSAICO_API bool mosaico_imu_fresh(const mosaico_imu_snapshot_t *s, int64_t now_us);
