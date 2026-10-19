#pragma once
#include <stddef.h>
#include <stdint.h>

#define DUCK_EYE_WIDTH 480
#define DUCK_EYE_HEIGHT 480

typedef struct {
    uint16_t count;
    uint16_t rgb565;
} duck_eye_run_t;

extern const duck_eye_run_t duck_eye_runs[];
extern const size_t duck_eye_run_count;
