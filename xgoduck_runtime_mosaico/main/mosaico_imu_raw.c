#include "mosaico_imu.h"

/* BMI270 0x0c..0x1a: accel XYZ, gyro XYZ, 24-bit sensor clock. No scaling or remap. */
void mosaico_imu_decode(const uint8_t data[15], int16_t acc[3], int16_t gyro[3], uint32_t *sensor_time) {
    for(int i=0;i<3;i++) {
        acc[i]=(int16_t)((uint16_t)data[2*i] | ((uint16_t)data[2*i+1]<<8));
        gyro[i]=(int16_t)((uint16_t)data[6+2*i] | ((uint16_t)data[7+2*i]<<8));
    }
    *sensor_time=(uint32_t)data[12] | ((uint32_t)data[13]<<8) | ((uint32_t)data[14]<<16);
}
