#include "policy.h"
#include "policy_layout.h"
#include <math.h>
static float dot_scalar(const float *w,const float *a,int n) {
    float sum=0;
    for(int i=0;i<n;i++) sum+=w[i]*a[i];
    return sum;
}
static float dot_unrolled(const float *w,const float *a,int n) {
    /* Independent accumulators shorten the floating-point dependency chain.
       Keep strict IEEE checks; do not enable -ffast-math. The 61-wide layer
       has a tail; the other layers have lengths divisible by four. */
    float s0=0,s1=0,s2=0,s3=0;int i=0;
    for(;i+3<n;i+=4) {
        s0+=w[i]*a[i];s1+=w[i+1]*a[i+1];
        s2+=w[i+2]*a[i+2];s3+=w[i+3]*a[i+3];
    }
    float sum=(s0+s1)+(s2+s3);
    for(;i<n;i++) sum+=w[i]*a[i];
    return sum;
}
static bool evaluate(const float *weights,const float obs[61],float out[14],float scratch[1024],bool optimized) {
    float *a=scratch,*b=scratch+512;
    for(int i=0;i<61;i++) {
        if(!isfinite(obs[i])) return false;
        a[i]=(obs[i]-weights[POLICY_MEAN+i])/weights[POLICY_DIV+i];
        if(!isfinite(a[i])) return false;
    }
    const int ins[4]={61,512,256,128}, outs[4]={512,256,128,14};
    const int offsets[4]={POLICY_W0,POLICY_W1,POLICY_W2,POLICY_W3};
    const int biases[4]={POLICY_B0,POLICY_B1,POLICY_B2,POLICY_B3};
    for(int layer=0;layer<4;layer++) {
        for(int row=0;row<outs[layer];row++) {
            const float *w=weights+offsets[layer]+row*ins[layer];
            float sum=optimized?dot_unrolled(w,a,ins[layer]):dot_scalar(w,a,ins[layer]);
            sum+=weights[biases[layer]+row];
            b[row]=(layer<3 && sum<0) ? expf(sum)-1 : sum;
            if(!isfinite(b[row])) return false;
        }
        float *tmp=a;a=b;b=tmp;
    }
    for(int i=0;i<14;i++) out[i]=a[i];
    return true;
}
bool policy_run_weights(const float *weights,const float obs[61],float out[14],float scratch[1024]) {
    return evaluate(weights,obs,out,scratch,true);
}
bool policy_run_reference(const float *weights,const float obs[61],float out[14],float scratch[1024]) {
    return evaluate(weights,obs,out,scratch,false);
}
bool policy_run(const float obs[61],float out[14],float scratch[1024]) {
    return policy_run_weights(policy_weights,obs,out,scratch);
}
