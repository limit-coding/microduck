#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifdef DUCK_HOST_TEST
#define DUCK_API __declspec(dllexport)
#else
#define DUCK_API
#endif
#define DUCK_FRAME 254
typedef struct {
    float acc[3], gyro[3], angles[15], vels[15];
    float gravity[3], gyro_rad[3];
    uint8_t battery;
} duck_state_t;
typedef struct {
    bool initialized;
    float gravity[3], gyro[3], vel[15];
} duck_filter_t;
typedef struct {
    bool initialized;
    float value[15];
} duck_output_filter_t;
typedef struct { uint8_t bytes[DUCK_FRAME]; size_t used; } duck_parser_t;
extern const float duck_default[15];
DUCK_API bool duck_parse_byte(duck_parser_t *p, uint8_t byte, duck_state_t *out);
DUCK_API bool duck_decode(const uint8_t payload[250], duck_state_t *out);
DUCK_API void duck_filter(duck_filter_t *f, duck_state_t *st, float dt);
DUCK_API void duck_observation(const duck_state_t *st, const float gyro_rad[3], const float gravity[3], const float prev[14], const float twist[3], const float head[4], float obs[61]);
DUCK_API void duck_targets(const float action[14], float targets[15]);
DUCK_API void duck_filter_targets(duck_output_filter_t *f, float targets[15]);
DUCK_API size_t duck_write(uint8_t main, const uint8_t *payload, size_t n, uint8_t *out);
DUCK_API size_t duck_angles_frame(const float targets[15], uint8_t out[83]);
