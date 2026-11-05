#include "audio_pwm.h"

#include <stdbool.h>

#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "hal/ledc_ll.h"
#include "soc/soc_caps.h"

#define AUDIO_PWM_DUTY_RESOLUTION LEDC_TIMER_8_BIT
#define AUDIO_PWM_DUTY_MIDPOINT 128U
/* The duty register has four fractional bits below the 8-bit duty. Levels are
 * kept in these Q4 units so a ramp moves by a fraction of an LSB per sample. */
#define AUDIO_PWM_DUTY_FRAC_BITS 4U
#define AUDIO_PWM_DUTY_MID_Q4 (AUDIO_PWM_DUTY_MIDPOINT << AUDIO_PWM_DUTY_FRAC_BITS)
/* 0 rounds every write to a whole LSB, to compare the ramps by ear. */
#ifndef AUDIO_PWM_USE_FRAC_DUTY
#define AUDIO_PWM_USE_FRAC_DUTY 1
#endif
#define AUDIO_PWM_TIMER LEDC_TIMER_3
#define AUDIO_PWM_CHANNEL LEDC_CHANNEL_7
/* The right channel of a stereo pair. LEDC timer 0 and channels 0-3 belong to
 * the general PWM port, so channel 6 on the audio timer is free. */
#define AUDIO_PWM_CHANNEL_RIGHT LEDC_CHANNEL_6
#if SOC_LEDC_SUPPORT_HS_MODE
#define AUDIO_PWM_MODE LEDC_HIGH_SPEED_MODE
#else
#define AUDIO_PWM_MODE LEDC_LOW_SPEED_MODE
#endif
#define AUDIO_PWM_GPTIMER_RESOLUTION_HZ 16000000U
#define AUDIO_PWM_GPTIMER_ALARM_TICKS \
    (AUDIO_PWM_GPTIMER_RESOLUTION_HZ / AUDIO_PWM_SAMPLE_RATE)
/* Mono holds 512 bytes (32 ms); stereo is two bytes per frame and gets twice
 * that, so both keep the same latency. */
#define AUDIO_PWM_BUFFER_BYTES 1024U
#define AUDIO_PWM_MONO_BUFFER_BYTES (AUDIO_PWM_BUFFER_BYTES / 2U)
#define AUDIO_PWM_WRITE_CHUNK 256U
/* Every stream fades up from 0 V to the midpoint before its first sample and
 * back down to 0 V after its last, so the output never steps. */
#define AUDIO_PWM_RAMP_MS 40U
#define AUDIO_PWM_RAMP_SAMPLES (AUDIO_PWM_SAMPLE_RATE * AUDIO_PWM_RAMP_MS / 1000U)
/* Close waits for the ramp-down while the sample interrupt makes progress.
 * Flash writes mask that interrupt, so a stall is counted in polls without
 * progress rather than wall-clock time; the cap bounds the whole wait. */
#define AUDIO_PWM_CLOSE_STALL_POLLS 20U
#define AUDIO_PWM_CLOSE_CAP_MS 2000U
#define AUDIO_PWM_PHASE_RAMP_UP 0U
#define AUDIO_PWM_PHASE_PLAY 1U
#define AUDIO_PWM_PHASE_RAMP_DOWN 2U
#define AUDIO_PWM_PHASE_DONE 3U

/* Output levels for the sample interrupt. Once the timer runs, only the
 * interrupt writes these fields except stop, which only the task writes. */
typedef struct {
    volatile uint8_t phase;
    volatile bool stop;
    /* Sample interrupts handled, so close can tell a stalled interrupt. */
    volatile uint32_t ticks;
    uint16_t pos;
    /* Last level written to each channel, Q4. */
    uint16_t level[2];
    /* Where each channel's ramp-down started, Q4. */
    uint16_t from[2];
} audio_pwm_shaper_t;

typedef struct {
    volatile bool active;
    /* Stereo frames are queued as a left byte then a right byte. */
    bool stereo;
    gpio_num_t pin;
    gpio_num_t pin_right;
    gptimer_handle_t sample_timer;
    StreamBufferHandle_t samples;
    StaticStreamBuffer_t samples_storage;
    uint8_t samples_buffer[AUDIO_PWM_BUFFER_BYTES];
    audio_pwm_shaper_t shaper;
} audio_pwm_state_t;

static audio_pwm_state_t audio_pwm;

/* Smoothstep from `from` to `to` in Q4 units: step n of `steps`. Monotonic,
 * exact at both ends, and at most about 1.5 * span / steps per step. */
static uint16_t IRAM_ATTR audio_pwm_ramp_q4(uint16_t from,
                                            uint16_t to,
                                            uint32_t n,
                                            uint32_t steps)
{
    if (n >= steps) {
        return to;
    }
    const uint32_t u = (n << 16) / steps;
    const uint32_t s = (uint32_t)(((uint64_t)(u * u) * ((3U << 16) - 2U * u)) >> 32);
    const uint32_t span = to >= from ? (uint32_t)(to - from) : (uint32_t)(from - to);
    const uint32_t d = (span * s + 0x8000U) >> 16;
    return (uint16_t)(to >= from ? from + d : from - d);
}

/* Advances the output by one sample. `got` says whether `in` holds a new
 * frame; the caller only reads the queue in PLAY. Stop is acted on once the
 * queue is empty, so queued audio plays out and a ramp-up always finishes. */
static void IRAM_ATTR audio_pwm_shaper_step(audio_pwm_shaper_t *sh,
                                            bool got,
                                            const uint8_t in[2])
{
    switch (sh->phase) {
    case AUDIO_PWM_PHASE_RAMP_UP:
        sh->pos++;
        sh->level[0] = audio_pwm_ramp_q4(0U,
                                         AUDIO_PWM_DUTY_MID_Q4,
                                         sh->pos,
                                         AUDIO_PWM_RAMP_SAMPLES);
        sh->level[1] = sh->level[0];
        if (sh->pos >= AUDIO_PWM_RAMP_SAMPLES) {
            sh->phase = AUDIO_PWM_PHASE_PLAY;
        }
        return;
    case AUDIO_PWM_PHASE_PLAY:
        if (got) {
            sh->level[0] = (uint16_t)(in[0] << AUDIO_PWM_DUTY_FRAC_BITS);
            sh->level[1] = (uint16_t)(in[1] << AUDIO_PWM_DUTY_FRAC_BITS);
            return;
        }
        if (!sh->stop) {
            /* Underrun: hold the last level. */
            return;
        }
        sh->from[0] = sh->level[0];
        sh->from[1] = sh->level[1];
        sh->pos = 0U;
        sh->phase = AUDIO_PWM_PHASE_RAMP_DOWN;
        __attribute__((fallthrough));
    case AUDIO_PWM_PHASE_RAMP_DOWN:
        sh->pos++;
        for (unsigned c = 0U; c < 2U; c++) {
            sh->level[c] = audio_pwm_ramp_q4(sh->from[c],
                                             0U,
                                             sh->pos,
                                             AUDIO_PWM_RAMP_SAMPLES);
        }
        if (sh->pos >= AUDIO_PWM_RAMP_SAMPLES) {
            sh->phase = AUDIO_PWM_PHASE_DONE;
        }
        return;
    default:
        return;
    }
}

static void IRAM_ATTR audio_pwm_set_duty_q4(ledc_channel_t channel, uint16_t q4)
{
    ledc_dev_t *hw = LEDC_LL_GET_HW();
    if (hw->channel_group[AUDIO_PWM_MODE]
            .channel[channel]
            .conf1.duty_start) {
        return;
    }
#if !AUDIO_PWM_USE_FRAC_DUTY
    q4 = (uint16_t)(((q4 + 8U) >> AUDIO_PWM_DUTY_FRAC_BITS) << AUDIO_PWM_DUTY_FRAC_BITS);
#endif
    /* The raw field takes the fraction too; ledc_ll_set_duty_int_part() would
     * shift it away. */
    hw->channel_group[AUDIO_PWM_MODE].channel[channel].duty.duty = q4;
    ledc_ll_set_duty_direction(hw,
                               AUDIO_PWM_MODE,
                               channel,
                               LEDC_DUTY_DIR_INCREASE);
    ledc_ll_set_duty_num(hw, AUDIO_PWM_MODE, channel, 1U);
    ledc_ll_set_duty_cycle(hw, AUDIO_PWM_MODE, channel, 1U);
    ledc_ll_set_duty_scale(hw, AUDIO_PWM_MODE, channel, 0U);
    ledc_ll_set_duty_start(hw, AUDIO_PWM_MODE, channel);
#if !SOC_LEDC_SUPPORT_HS_MODE
    ledc_ll_ls_channel_update(hw, AUDIO_PWM_MODE, channel);
#endif
}

static bool IRAM_ATTR audio_pwm_on_sample(
    gptimer_handle_t timer,
    const gptimer_alarm_event_data_t *event,
    void *user)
{
    (void)timer;
    (void)event;
    audio_pwm_state_t *state = user;
    audio_pwm_shaper_t *shaper = &state->shaper;
    if (shaper->phase == AUDIO_PWM_PHASE_DONE) {
        return false;
    }
    uint8_t in[2] = {0U, 0U};
    bool got = false;
    BaseType_t task_woken = pdFALSE;
    if (shaper->phase == AUDIO_PWM_PHASE_PLAY && state->samples != NULL) {
        const size_t want = state->stereo ? 2U : 1U;
        got = xStreamBufferReceiveFromISR(state->samples,
                                          in,
                                          want,
                                          &task_woken) == want;
        if (!state->stereo) {
            in[1] = in[0];
        }
    }
    audio_pwm_shaper_step(shaper, got, in);
    shaper->ticks++;
    audio_pwm_set_duty_q4(AUDIO_PWM_CHANNEL, shaper->level[0]);
    if (state->stereo) {
        audio_pwm_set_duty_q4(AUDIO_PWM_CHANNEL_RIGHT, shaper->level[1]);
    }
    return task_woken == pdTRUE;
}

static void audio_pwm_park_pin(gpio_num_t pin)
{
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) {
        return;
    }
    /* The hold freezes the pad while it moves from the LEDC (or its reset
     * state) to a plain low output, so the reset's pull-up never shows. Pads
     * without hold support return ESP_ERR_NOT_SUPPORTED and are only driven. */
    (void)gpio_hold_en(pin);
    (void)gpio_reset_pin(pin);
    (void)gpio_pullup_dis(pin);
    (void)gpio_set_level(pin, 0);
    (void)gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    /* Otherwise light sleep switches the pad to its floating sleep state. */
    (void)gpio_sleep_sel_dis(pin);
    (void)gpio_hold_dis(pin);
    (void)gpio_hold_en(pin);
}

void audio_pwm_park_pins(gpio_num_t left, gpio_num_t right)
{
    audio_pwm_park_pin(left);
    audio_pwm_park_pin(right);
}

void audio_pwm_release_pins(gpio_num_t left, gpio_num_t right)
{
    if (GPIO_IS_VALID_OUTPUT_GPIO(left)) {
        (void)gpio_hold_dis(left);
    }
    if (GPIO_IS_VALID_OUTPUT_GPIO(right)) {
        (void)gpio_hold_dis(right);
    }
}

static void audio_pwm_stop_timer(void)
{
    if (audio_pwm.sample_timer == NULL) {
        return;
    }
    (void)gptimer_stop(audio_pwm.sample_timer);
    (void)gptimer_disable(audio_pwm.sample_timer);
    (void)gptimer_del_timer(audio_pwm.sample_timer);
    audio_pwm.sample_timer = NULL;
}

static void audio_pwm_stop_channels(void)
{
    (void)ledc_stop(AUDIO_PWM_MODE, AUDIO_PWM_CHANNEL, 0U);
    if (audio_pwm.stereo) {
        (void)ledc_stop(AUDIO_PWM_MODE, AUDIO_PWM_CHANNEL_RIGHT, 0U);
    }
}

static esp_err_t audio_pwm_start_timer(void)
{
    const gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = AUDIO_PWM_GPTIMER_RESOLUTION_HZ,
    };
    esp_err_t err = gptimer_new_timer(&timer_config, &audio_pwm.sample_timer);
    if (err != ESP_OK) {
        return err;
    }
    const gptimer_event_callbacks_t callbacks = {
        .on_alarm = audio_pwm_on_sample,
    };
    err = gptimer_register_event_callbacks(audio_pwm.sample_timer,
                                           &callbacks,
                                           &audio_pwm);
    if (err == ESP_OK) {
        const gptimer_alarm_config_t alarm = {
            .alarm_count = AUDIO_PWM_GPTIMER_ALARM_TICKS,
            .reload_count = 0U,
            .flags.auto_reload_on_alarm = true,
        };
        err = gptimer_set_alarm_action(audio_pwm.sample_timer, &alarm);
    }
    if (err == ESP_OK) {
        err = gptimer_enable(audio_pwm.sample_timer);
    }
    if (err == ESP_OK) {
        audio_pwm.active = true;
        err = gptimer_start(audio_pwm.sample_timer);
    }
    return err;
}

static esp_err_t audio_pwm_open_pins(gpio_num_t pin, gpio_num_t pin_right, bool stereo)
{
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin) ||
        (stereo && (!GPIO_IS_VALID_OUTPUT_GPIO(pin_right) || pin_right == pin))) {
        return ESP_ERR_INVALID_ARG;
    }
    if (audio_pwm.active) {
        return ESP_ERR_INVALID_STATE;
    }

    /* The queue is recreated for each open so mono and stereo get their own
     * capacity from the same storage. */
    if (audio_pwm.samples != NULL) {
        vStreamBufferDelete(audio_pwm.samples);
        audio_pwm.samples = NULL;
    }
    audio_pwm.samples = xStreamBufferCreateStatic(
        stereo ? sizeof(audio_pwm.samples_buffer) : AUDIO_PWM_MONO_BUFFER_BYTES,
        1U,
        audio_pwm.samples_buffer,
        &audio_pwm.samples_storage);
    if (audio_pwm.samples == NULL) {
        return ESP_ERR_NO_MEM;
    }
    audio_pwm.stereo = stereo;
    audio_pwm.pin = pin;
    audio_pwm.pin_right = stereo ? pin_right : GPIO_NUM_NC;

    const ledc_timer_config_t ledc_timer = {
        .speed_mode = AUDIO_PWM_MODE,
        .duty_resolution = AUDIO_PWM_DUTY_RESOLUTION,
        .timer_num = AUDIO_PWM_TIMER,
        .freq_hz = AUDIO_PWM_CARRIER_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&ledc_timer),
                        "audio_pwm",
                        "LEDC timer config failed");

    /*
     * ESP-IDF configures a channel's initial duty before binding the timer
     * requested by ledc_channel_config().  On classic ESP32 the channel still
     * points at unconfigured timer 0 here, so the duty-start handshake never
     * completes and ledc_update_duty() holds the spinlock until the interrupt
     * watchdog fires.  Bind our running carrier timer first.
     */
    ESP_RETURN_ON_ERROR(ledc_bind_channel_timer(AUDIO_PWM_MODE,
                                                AUDIO_PWM_CHANNEL,
                                                AUDIO_PWM_TIMER),
                        "audio_pwm",
                        "LEDC channel timer bind failed");

    /* Duty 0 matches the parked low level; the sample interrupt ramps up. */
    const ledc_channel_config_t ledc_channel = {
        .gpio_num = pin,
        .speed_mode = AUDIO_PWM_MODE,
        .channel = AUDIO_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = AUDIO_PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };
    esp_err_t err = ledc_channel_config(&ledc_channel);
    if (err == ESP_OK && stereo) {
        err = ledc_bind_channel_timer(AUDIO_PWM_MODE,
                                      AUDIO_PWM_CHANNEL_RIGHT,
                                      AUDIO_PWM_TIMER);
        if (err != ESP_OK) {
            ESP_LOGE("audio_pwm", "LEDC right channel timer bind failed");
        } else {
            ledc_channel_config_t right = ledc_channel;
            right.gpio_num = pin_right;
            right.channel = AUDIO_PWM_CHANNEL_RIGHT;
            err = ledc_channel_config(&right);
        }
    }
    if (err == ESP_OK) {
        audio_pwm.shaper = (audio_pwm_shaper_t){.phase = AUDIO_PWM_PHASE_RAMP_UP};
        /* The pads go from held low to LEDC low, so releasing does not step. */
        audio_pwm_release_pins(audio_pwm.pin, audio_pwm.pin_right);
        err = audio_pwm_start_timer();
    }
    if (err != ESP_OK) {
        audio_pwm.active = false;
        audio_pwm_stop_timer();
        audio_pwm_stop_channels();
        audio_pwm_park_pins(audio_pwm.pin, audio_pwm.pin_right);
        return err;
    }
    return ESP_OK;
}

esp_err_t audio_pwm_open(gpio_num_t pin)
{
    return audio_pwm_open_pins(pin, GPIO_NUM_NC, false);
}

esp_err_t audio_pwm_open_stereo(gpio_num_t left, gpio_num_t right)
{
    return audio_pwm_open_pins(left, right, true);
}

esp_err_t audio_pwm_write_s16(const int16_t *samples,
                              size_t frames,
                              uint8_t channels,
                              uint8_t volume)
{
    if (samples == NULL || frames == 0U || channels == 0U || channels > 2U ||
        volume > 100U) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!audio_pwm.active || audio_pwm.samples == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* A stereo frame queues two bytes, so a chunk holds half as many frames and
     * the byte count per send stays the same. */
    const size_t bytes_per_frame = audio_pwm.stereo ? 2U : 1U;
    const size_t chunk_frames = AUDIO_PWM_WRITE_CHUNK / bytes_per_frame;
    uint8_t duties[AUDIO_PWM_WRITE_CHUNK];
    size_t consumed = 0U;
    while (consumed < frames) {
        const size_t count = frames - consumed < chunk_frames ?
            frames - consumed : chunk_frames;
        for (size_t i = 0; i < count; i++) {
            const size_t frame = consumed + i;
            int32_t left = samples[frame * channels];
            int32_t right = channels == 2U ? samples[(frame * channels) + 1U] : left;
            if (audio_pwm.stereo) {
                left = (left * volume) / 100;
                right = (right * volume) / 100;
                duties[i * 2U] = (uint8_t)((left + 32768) >> 8);
                duties[(i * 2U) + 1U] = (uint8_t)((right + 32768) >> 8);
            } else {
                int32_t sample = left;
                if (channels == 2U) {
                    sample = (left + right) / 2;
                }
                sample = (sample * volume) / 100;
                duties[i] = (uint8_t)((sample + 32768) >> 8);
            }
        }
        /* Blocking sends wake on every sample interrupt. */
        const size_t bytes = count * bytes_per_frame;
        size_t available;
        while ((available = xStreamBufferSpacesAvailable(audio_pwm.samples)) < bytes) {
            const TickType_t ticks = pdMS_TO_TICKS(
                ((bytes - available) / bytes_per_frame) * 1000U / AUDIO_PWM_SAMPLE_RATE);
            vTaskDelay(ticks ? ticks : 1U);
        }
        const size_t sent = xStreamBufferSend(audio_pwm.samples,
                                              duties,
                                              bytes,
                                              0U);
        if (sent != bytes) {
            return ESP_ERR_INVALID_SIZE;
        }
        consumed += count;
    }
    return ESP_OK;
}

/* Lets the interrupt drain the queue and ramp down. Gives up when the
 * interrupt makes no progress for a while or the cap passes; the output then
 * steps to 0 V when the channels stop. */
static void audio_pwm_wait_ramp_down(void)
{
    const TickType_t start = xTaskGetTickCount();
    uint32_t last = audio_pwm.shaper.ticks;
    uint32_t stalled = 0U;
    audio_pwm.shaper.stop = true;
    while (audio_pwm.shaper.phase != AUDIO_PWM_PHASE_DONE) {
        if (stalled >= AUDIO_PWM_CLOSE_STALL_POLLS ||
            xTaskGetTickCount() - start >= pdMS_TO_TICKS(AUDIO_PWM_CLOSE_CAP_MS)) {
            ESP_LOGW("audio_pwm", "output stopped before its ramp-down finished");
            return;
        }
        vTaskDelay(1U);
        const uint32_t ticks = audio_pwm.shaper.ticks;
        stalled = ticks == last ? stalled + 1U : 0U;
        last = ticks;
    }
}

void audio_pwm_close(void)
{
    if (!audio_pwm.active && audio_pwm.sample_timer == NULL) {
        return;
    }
    if (audio_pwm.sample_timer != NULL) {
        audio_pwm_wait_ramp_down();
    }
    audio_pwm.active = false;
    audio_pwm_stop_timer();
    audio_pwm_stop_channels();
    audio_pwm_park_pins(audio_pwm.pin, audio_pwm.pin_right);
    if (audio_pwm.samples != NULL) {
        (void)xStreamBufferReset(audio_pwm.samples);
    }
}
