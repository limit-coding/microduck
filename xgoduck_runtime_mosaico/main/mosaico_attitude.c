#include "mosaico_imu.h"
#include <math.h>
#include <string.h>

static float norm3(const float v[3]) {
    return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
}
bool mosaico_estimate(mosaico_estimator_t *f,const int16_t acc[3],const int16_t gyro[3],float dt,
                     float acc_g[3],float gyro_rad[3],float gravity[3]) {
    const float ascale=4.0f/32768.0f;
    const float wscale=(2000.0f/32768.0f)*(3.14159265358979323846f/180.0f);
    /* Exactly the user-specified mappings, with floating-point negation (INT16_MIN is safe). */
    acc_g[0]=acc[2]*ascale;acc_g[1]=acc[1]*ascale;acc_g[2]=acc[0]*ascale;
    gyro_rad[0]=gyro[2]*wscale;gyro_rad[1]=-gyro[1]*wscale;gyro_rad[2]=-gyro[0]*wscale;
    if(!isfinite(dt) || dt<=0) {f->initialized=false;return false;}
    float anorm=norm3(acc_g);
    /* A resting accelerometer measures support force, opposite to downward gravity. */
    float measured[3]={0};
    if(anorm>1e-6f) for(int i=0;i<3;i++) measured[i]=-acc_g[i]/anorm;
    if(!f->initialized || dt>0.1f) {
        f->initialized=false;
        if(anorm<0.8f || anorm>1.2f) return false;
        memcpy(f->gravity,measured,sizeof(measured));f->initialized=true;
    } else {
        float rate=norm3(gyro_rad),predicted[3];
        if(rate>1e-6f) {
            float u[3]={gyro_rad[0]/rate,gyro_rad[1]/rate,gyro_rad[2]/rate};
            const float *g=f->gravity;
            float c=cosf(rate*dt),s=sinf(rate*dt),dot=u[0]*g[0]+u[1]*g[1]+u[2]*g[2];
            /* World-fixed gravity in body coordinates: dg/dt = g cross omega. */
            predicted[0]=c*g[0]+s*(g[1]*u[2]-g[2]*u[1])+(1-c)*dot*u[0];
            predicted[1]=c*g[1]+s*(g[2]*u[0]-g[0]*u[2])+(1-c)*dot*u[1];
            predicted[2]=c*g[2]+s*(g[0]*u[1]-g[1]*u[0])+(1-c)*dot*u[2];
        } else memcpy(predicted,f->gravity,sizeof(predicted));
        float trust=fmaxf(0.0f,1.0f-fabsf(anorm-1.0f)/0.2f);
        float gain=(1.0f-expf(-dt/0.3f))*trust;
        for(int i=0;i<3;i++) f->gravity[i]=(1-gain)*predicted[i]+gain*measured[i];
        float n=norm3(f->gravity);
        if(!isfinite(n) || n<1e-6f) {f->initialized=false;return false;}
        for(int i=0;i<3;i++) f->gravity[i]/=n;
    }
    /* User-requested final output signs; leave the estimator's internal frame unchanged. */
    for(int i=0;i<3;i++) gravity[i]=-f->gravity[i];
    gyro_rad[0]=-gyro_rad[0];
    return true;
}
bool mosaico_imu_fresh(const mosaico_imu_snapshot_t *s,int64_t now_us) {
    return s->ready && s->estimate_valid && s->error==0 && s->samples &&
           now_us>=s->sample_us && now_us-s->sample_us<=MOSAICO_IMU_MAX_AGE_US;
}
