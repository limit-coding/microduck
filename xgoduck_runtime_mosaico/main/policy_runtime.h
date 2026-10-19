#pragma once
#include "policy.h"
typedef struct {
    bool psram;
    float flash_ms, ram_scalar_ms, optimized_ms, max_error;
} policy_runtime_stats_t;
/* Control-task-only initialization; keeps the copied weights for the boot lifetime. */
bool policy_runtime_init(float scratch[1024],policy_runtime_stats_t *stats);
bool policy_runtime_run(const float obs[61],float out[14],float scratch[1024]);
