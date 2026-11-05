#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#define AUDIO_PWM_SAMPLE_RATE 16000U
#define AUDIO_PWM_CARRIER_HZ 78125U
#define AUDIO_PWM_FRAMES_PER_BLOCK 256U

/* Mono output on one pin. Each stream ramps up from 0 V before its first
 * sample and back down after its last; close blocks until that ramp ends. */
esp_err_t audio_pwm_open(gpio_num_t pin);
/* Stereo output: left and right on separate pins, sharing one carrier timer.
 * Mono input is played on both channels. */
esp_err_t audio_pwm_open_stereo(gpio_num_t left, gpio_num_t right);
esp_err_t audio_pwm_write_s16(const int16_t *samples,
                              size_t frames,
                              uint8_t channels,
                              uint8_t volume);
/* Drains queued audio, ramps down, then parks the pins. */
void audio_pwm_close(void);
/* Drives the pins low and latches them with the pad hold, so they stay low
 * between streams, in light sleep, across CPU resets and, on RTC pads, in deep
 * sleep. Open releases the hold and close parks again. GPIO_NUM_NC is skipped. */
void audio_pwm_park_pins(gpio_num_t left, gpio_num_t right);
/* Releases the hold; the pins stay driven low until reconfigured. */
void audio_pwm_release_pins(gpio_num_t left, gpio_num_t right);
