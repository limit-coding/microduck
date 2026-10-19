#include "policy_runtime.h"
#include "policy_layout.h"
#include "policy_reference.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const float *active_weights=policy_weights;
static bool initialized;
bool policy_runtime_run(const float obs[61],float out[14],float scratch[1024]) {
    return initialized && policy_run_weights(active_weights,obs,out,scratch);
}
bool policy_runtime_init(float scratch[1024],policy_runtime_stats_t *stats) {
    memset(stats,0,sizeof(*stats));
    float *ram=heap_caps_aligned_alloc(64,POLICY_FLOATS*sizeof(float),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if(ram) {
        memcpy(ram,policy_weights,POLICY_FLOATS*sizeof(float));
        if(memcmp(ram,policy_weights,POLICY_FLOATS*sizeof(float))!=0) {
            heap_caps_free(ram);return false;
        }
        active_weights=ram;stats->psram=true;
    }
    /* Compare three identical inputs across flash/scalar, RAM/scalar and RAM/
       unrolled. This executes only before inference can be enabled. Timings
       describe the actual chip, and are never substituted for control timings. */
    float references[3][14]={{0}},obs[61],out[14]={0};
    bool ok=true;
    for(int pass=0;pass<3;pass++) {
        int64_t elapsed=0;
        for(int sample=0;sample<3;sample++) {
            for(int i=0;i<61;i++) obs[i]=sample==0?0:((i%11)-5)*0.03f*sample;
            int64_t started=esp_timer_get_time();
            bool finite=pass==0?policy_run_reference(policy_weights,obs,out,scratch):
                        pass==1?policy_run_reference(active_weights,obs,out,scratch):
                                policy_run_weights(active_weights,obs,out,scratch);
            elapsed+=esp_timer_get_time()-started;
            if(!finite) ok=false;
            for(int i=0;i<14;i++) {
                if(sample==0 && (!isfinite(out[i]) || fabsf(out[i]-policy_zero_reference[i])>0.0002f)) ok=false;
                if(pass==0) references[sample][i]=out[i];
                else {
                    float error=fabsf(out[i]-references[sample][i]);
                    if(!isfinite(out[i]) || error>0.0002f) ok=false;
                    if(error>stats->max_error) stats->max_error=error;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        float ms=elapsed/3000.0f;
        if(pass==0) stats->flash_ms=ms;
        else if(pass==1) stats->ram_scalar_ms=ms;
        else stats->optimized_ms=ms;
    }
    if(!ok) {
        if(ram) heap_caps_free(ram);
        active_weights=policy_weights;stats->psram=false;
        return false;
    }
    initialized=true;
    return true;
}
