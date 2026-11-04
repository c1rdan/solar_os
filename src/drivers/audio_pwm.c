#include "audio_pwm.h"

#include <stdbool.h>

#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "hal/ledc_ll.h"
#include "soc/soc_caps.h"

#define AUDIO_PWM_DUTY_RESOLUTION LEDC_TIMER_8_BIT
#define AUDIO_PWM_DUTY_MIDPOINT 128U
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
#define AUDIO_PWM_CLOSE_DRAIN_MS 40U

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
} audio_pwm_state_t;

static audio_pwm_state_t audio_pwm;

static void IRAM_ATTR audio_pwm_set_duty(ledc_channel_t channel, uint8_t duty)
{
    ledc_dev_t *hw = LEDC_LL_GET_HW();
    if (hw->channel_group[AUDIO_PWM_MODE]
            .channel[channel]
            .conf1.duty_start) {
        return;
    }
    ledc_ll_set_duty_int_part(hw, AUDIO_PWM_MODE, channel, duty);
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
    uint8_t duty[2] = {AUDIO_PWM_DUTY_MIDPOINT, AUDIO_PWM_DUTY_MIDPOINT};
    BaseType_t task_woken = pdFALSE;
    if (state->active && state->samples != NULL) {
        const size_t want = state->stereo ? 2U : 1U;
        if (xStreamBufferReceiveFromISR(state->samples,
                                        duty,
                                        want,
                                        &task_woken) != want) {
            /* Underrun: hold the idle level on both channels. */
            duty[0] = AUDIO_PWM_DUTY_MIDPOINT;
            duty[1] = AUDIO_PWM_DUTY_MIDPOINT;
        }
    }
    audio_pwm_set_duty(AUDIO_PWM_CHANNEL, duty[0]);
    if (state->stereo) {
        audio_pwm_set_duty(AUDIO_PWM_CHANNEL_RIGHT, duty[1]);
    }
    return task_woken == pdTRUE;
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

    const ledc_channel_config_t ledc_channel = {
        .gpio_num = pin,
        .speed_mode = AUDIO_PWM_MODE,
        .channel = AUDIO_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = AUDIO_PWM_TIMER,
        .duty = AUDIO_PWM_DUTY_MIDPOINT,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };
    esp_err_t err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        return err;
    }
    if (stereo) {
        ESP_RETURN_ON_ERROR(ledc_bind_channel_timer(AUDIO_PWM_MODE,
                                                    AUDIO_PWM_CHANNEL_RIGHT,
                                                    AUDIO_PWM_TIMER),
                            "audio_pwm",
                            "LEDC right channel timer bind failed");
        ledc_channel_config_t right = ledc_channel;
        right.gpio_num = pin_right;
        right.channel = AUDIO_PWM_CHANNEL_RIGHT;
        err = ledc_channel_config(&right);
        if (err != ESP_OK) {
            (void)ledc_stop(AUDIO_PWM_MODE, AUDIO_PWM_CHANNEL, 0U);
            return err;
        }
    }

    const gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = AUDIO_PWM_GPTIMER_RESOLUTION_HZ,
    };
    err = gptimer_new_timer(&timer_config, &audio_pwm.sample_timer);
    if (err != ESP_OK) {
        audio_pwm_stop_channels();
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
        audio_pwm.pin = pin;
        audio_pwm.pin_right = stereo ? pin_right : GPIO_NUM_NC;
        audio_pwm.active = true;
        err = gptimer_start(audio_pwm.sample_timer);
    }
    if (err != ESP_OK) {
        audio_pwm.active = false;
        audio_pwm_stop_timer();
        audio_pwm_stop_channels();
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

void audio_pwm_close(void)
{
    if (!audio_pwm.active && audio_pwm.sample_timer == NULL) {
        return;
    }
    if (audio_pwm.active && audio_pwm.samples != NULL) {
        TickType_t drain_ticks = pdMS_TO_TICKS(AUDIO_PWM_CLOSE_DRAIN_MS);
        if (drain_ticks == 0U) {
            drain_ticks = 1U;
        }
        for (TickType_t waited = 0U;
             waited < drain_ticks &&
             xStreamBufferBytesAvailable(audio_pwm.samples) != 0U;
             waited++) {
            vTaskDelay(1U);
        }
    }
    audio_pwm.active = false;
    audio_pwm_stop_timer();
    (void)ledc_set_duty(AUDIO_PWM_MODE,
                        AUDIO_PWM_CHANNEL,
                        AUDIO_PWM_DUTY_MIDPOINT);
    (void)ledc_update_duty(AUDIO_PWM_MODE, AUDIO_PWM_CHANNEL);
    if (audio_pwm.stereo) {
        (void)ledc_set_duty(AUDIO_PWM_MODE,
                            AUDIO_PWM_CHANNEL_RIGHT,
                            AUDIO_PWM_DUTY_MIDPOINT);
        (void)ledc_update_duty(AUDIO_PWM_MODE, AUDIO_PWM_CHANNEL_RIGHT);
    }
    audio_pwm_stop_channels();
    if (audio_pwm.samples != NULL) {
        (void)xStreamBufferReset(audio_pwm.samples);
    }
}
