#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"

typedef struct {
    rmt_channel_handle_t channel;
    rmt_encoder_handle_t encoder;
    int data_pin;
    bool enabled; /* RMT channel enabled; only while a frame is sent */
} neopixel_t;

esp_err_t neopixel_init(neopixel_t *strip, int data_pin);
esp_err_t neopixel_write(neopixel_t *strip, const uint8_t *grb, size_t byte_count);
void neopixel_deinit(neopixel_t *strip);
