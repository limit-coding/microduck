#pragma once

#include "esp_err.h"

/* Initialize the onboard CO5300, draw the eye once, then leave the bus idle. */
esp_err_t duck_display_show_static(void);
