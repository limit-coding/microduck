#include "duck_core.h"
#include <math.h>
#include <string.h>
static const float RAD = 0.017453292519943295f;
static const uint8_t motor_ids[15] = {10,11,12,13,14,20,21,22,23,24,30,31,32,33,34};
static const uint8_t fb_to_motor[15] = {0,1,2,3,4,10,11,12,13,14,5,6,7,8,9};
static const uint8_t policy_index[14] = {0,1,2,3,4,10,11,12,13,5,6,7,8,9};
/* Training observation reference stays fixed; output pose is configured separately. */
static const float policy_home[14] = {0,-5,-22,0,22,
                                    25,25,0,0,
                                    0,5,22,0,-22}; 

const float duck_default[15] = {0,-5,-22,0,22,
                                0, 5,22,0,-22,
                                25,25,0,0,0};
static float read_float(const uint8_t *p) { float f; memcpy(&f,p,4); return f; }
bool duck_decode(const uint8_t p[250], duck_state_t *st) {
    float imu[9], angle[15], vel[15];
    for(int i=0;i<9;i++) {
        imu[i]=read_float(p+20+4*i);
        if(!isfinite(imu[i])) return false;
    }
    for(int i=0;i<15;i++) {
        angle[i]=read_float(p+56+4*i); vel[i]=read_float(p+120+4*i);
        if(!isfinite(angle[i]) || !isfinite(vel[i])) return false;
    }
    memset(st,0,sizeof(*st)); st->battery=p[0]; st->gravity[2]=-1;
    memcpy(st->acc,imu+3,12); memcpy(st->gyro,imu+6,12);
    for(int i=0;i<15;i++) { st->angles[i]=angle[fb_to_motor[i]]; st->vels[i]=vel[fb_to_motor[i]]; }
    return true;
}
bool duck_parse_byte(duck_parser_t *p, uint8_t b, duck_state_t *out) {
    p->bytes[p->used++]=b;
    while(p->used) {
        if(p->bytes[0]!=0x55 || (p->used>1 && p->bytes[1]!=0)) {
            memmove(p->bytes,p->bytes+1,--p->used); continue;
        }
        if(p->used<DUCK_FRAME) return false;
        if(p->bytes[252]==0 && p->bytes[253]==0xaa) {
            bool ok=duck_decode(p->bytes+2,out); p->used=0; return ok;
        }
        memmove(p->bytes,p->bytes+1,--p->used);
    }
    return false;
}
void duck_filter(duck_filter_t *f, duck_state_t *st, float dt) {
    float acc[3]={st->acc[2],-st->acc[0],-st->acc[1]};
    float gyro[3]={st->gyro[2],-st->gyro[0],-st->gyro[1]};
    float anorm=sqrtf(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);
    bool reset=!f->initialized || dt>0.1f;
    dt=fmaxf(0.001f,fminf(dt,0.1f));
    if(reset) {
        if(anorm>1e-6f) for(int i=0;i<3;i++) f->gravity[i]=acc[i]/anorm;
        else {f->gravity[0]=0;f->gravity[1]=0;f->gravity[2]=-1;}
        memcpy(f->gyro,gyro,sizeof(f->gyro));
        memcpy(f->vel,st->vels,sizeof(f->vel));
        f->initialized=true;
    } else {
        float gx=f->gravity[0],gy=f->gravity[1],gz=f->gravity[2];
        float wx=gyro[0]*RAD,wy=gyro[1]*RAD,wz=gyro[2]*RAD;
        float predicted[3]={gx+(wz*gy-wy*gz)*dt,
                            gy+(wx*gz-wz*gx)*dt,
                            gz+(wy*gx-wx*gy)*dt};
        float pnorm=sqrtf(predicted[0]*predicted[0]+predicted[1]*predicted[1]+predicted[2]*predicted[2]);
        for(int i=0;i<3;i++) predicted[i]/=pnorm;
        float trust=fmaxf(0.0f,1.0f-fabsf(anorm/9.8f-1.0f)/0.2f);
        float gain=(1.0f-expf(-dt/0.3f))*trust;
        if(anorm>1e-6f) for(int i=0;i<3;i++) predicted[i]=(1-gain)*predicted[i]+gain*acc[i]/anorm;
        pnorm=sqrtf(predicted[0]*predicted[0]+predicted[1]*predicted[1]+predicted[2]*predicted[2]);
        for(int i=0;i<3;i++) f->gravity[i]=predicted[i]/pnorm;
        float alpha=powf(0.3f,dt/0.01f);
        for(int i=0;i<3;i++) f->gyro[i]=alpha*f->gyro[i]+(1-alpha)*gyro[i];
        for(int i=0;i<15;i++) f->vel[i]=alpha*f->vel[i]+(1-alpha)*st->vels[i];
    }
    memcpy(st->gravity,f->gravity,sizeof(f->gravity));
    for(int i=0;i<3;i++) st->gyro_rad[i]=f->gyro[i]*RAD;
    memcpy(st->vels,f->vel,sizeof(f->vel));
}
void duck_observation(const duck_state_t *st,const float gyro_rad[3],const float gravity[3],const float prev[14],const float twist[3],const float head[4],float obs[61]) {
    memset(obs,0,61*sizeof(float)); memcpy(obs,gyro_rad,12); memcpy(obs+3,gravity,12);
    for(int i=0;i<14;i++) { obs[6+i]=(st->angles[policy_index[i]]-policy_home[i])*RAD; obs[20+i]=st->vels[policy_index[i]]*RAD; }
    memcpy(obs+34,prev,56); memcpy(obs+48,twist,12); memcpy(obs+51,head,16);
}
void duck_targets(const float action[14],float targets[15]) {
    memset(targets,0,15*sizeof(float));
    for(int i=0;i<14;i++) targets[policy_index[i]]=duck_default[policy_index[i]]+action[i]*(180.0f/3.14159265358979323846f);
}
void duck_filter_targets(duck_output_filter_t *f,float targets[15]) {
    if(!f->initialized) {
        memcpy(f->value,targets,sizeof(f->value));
        f->initialized=true;
        return;
    }
    for(int i=0;i<15;i++) {
        targets[i]=0.3f*f->value[i]+0.7f*targets[i];
        f->value[i]=targets[i];
    }
}
size_t duck_write(uint8_t main,const uint8_t *payload,size_t n,uint8_t *out) {
    if(n>247) return 0;
    out[0]=0x55;out[1]=0;out[2]=(uint8_t)(n+8);out[3]=3;out[4]=main;
    memcpy(out+5,payload,n); uint8_t sum=0;
    for(size_t i=2;i<n+5;i++) sum+=out[i];
    out[n+5]=(uint8_t)~sum;out[n+6]=0;out[n+7]=0xaa;return n+8;
}
size_t duck_angles_frame(const float targets[15],uint8_t out[83]) {
    uint8_t p[75];
    for(int i=0;i<15;i++) { p[i*5]=motor_ids[i];memcpy(p+i*5+1,targets+i,4); }
    return duck_write(40,p,75,out);
}
