#pragma once
#include "duck_core.h"
DUCK_API extern const float policy_weights[];
/* Caller owns 2 x 512 float work space; no allocation or global mutable inference state. */
DUCK_API bool policy_run(const float obs[61], float out[14], float scratch[1024]);
/* Explicit immutable weight storage: flash on host, PSRAM after device initialization. */
DUCK_API bool policy_run_weights(const float *weights, const float obs[61], float out[14], float scratch[1024]);
/* Original scalar accumulation, retained for startup A/B timing and numerical checks. */
DUCK_API bool policy_run_reference(const float *weights, const float obs[61], float out[14], float scratch[1024]);
