#include "solar_os_audio_pwm.h"

#include <stdio.h>
#include <string.h>

#include "audio_pwm.h"
#include "esp_log.h"
#include "solar_os_audio.h"
#include "solar_os_stream.h"

#define AUDIO_PWM_DEVICE_NAME_MAX 16U

static const char *TAG = "audio-pwm";

typedef struct {
    bool attached;
    int pin;
    /* Right channel pin of a stereo pair, or -1 for mono. */
    int pin_right;
    solar_os_stream_audio_format_t native_format;
    uint8_t volume;
    char id[SOLAR_OS_AUDIO_DEVICE_ID_MAX];
    char stream_id[SOLAR_OS_STREAM_ID_MAX];
} solar_os_audio_pwm_device_t;

static solar_os_audio_pwm_device_t pwm_audio;

static solar_os_stream_audio_format_t pwm_audio_native_format_for(bool stereo)
{
    return (solar_os_stream_audio_format_t) {
        .sample_format = SOLAR_OS_STREAM_AUDIO_S16_LE,
        .sample_rate = AUDIO_PWM_SAMPLE_RATE,
        .channels = stereo ? 2U : 1U,
        .bits_per_sample = 16U,
        .frames_per_block = AUDIO_PWM_FRAMES_PER_BLOCK,
    };
}

static bool audio_pwm_requested_format_supported(
    const solar_os_stream_audio_format_t *format)
{
    if (format == NULL) {
        return true;
    }
    return (format->sample_rate == 0U ||
            format->sample_rate == AUDIO_PWM_SAMPLE_RATE) &&
        (format->channels == 0U || format->channels == 1U ||
         format->channels == 2U) &&
        (format->bits_per_sample == 0U || format->bits_per_sample == 16U) &&
        format->sample_format == SOLAR_OS_STREAM_AUDIO_S16_LE;
}

static esp_err_t audio_pwm_stream_open(
    void *user,
    const char *owner,
    const solar_os_stream_open_options_t *options,
    solar_os_stream_handle_t *handle)
{
    (void)owner;
    solar_os_audio_pwm_device_t *device = user;
    if (device == NULL || !device->attached || handle == NULL ||
        !audio_pwm_requested_format_supported(
            options != NULL ? &options->requested_audio : NULL)) {
        return device != NULL && device->attached ?
            ESP_ERR_NOT_SUPPORTED : ESP_ERR_INVALID_STATE;
    }

    solar_os_stream_audio_format_t format = device->native_format;
    if (options != NULL) {
        if (options->requested_audio.sample_rate != 0U) {
            format.sample_rate = options->requested_audio.sample_rate;
        }
        if (options->requested_audio.channels != 0U) {
            format.channels = options->requested_audio.channels;
        }
    }
    const esp_err_t err = device->pin_right >= 0 ?
        audio_pwm_open_stereo((gpio_num_t)device->pin, (gpio_num_t)device->pin_right) :
        audio_pwm_open((gpio_num_t)device->pin);
    if (err == ESP_OK) {
        handle->context = device;
        handle->audio = format;
    }
    return err;
}

static void audio_pwm_stream_close(void *user, solar_os_stream_handle_t *handle)
{
    (void)user;
    audio_pwm_close();
    handle->context = NULL;
}

static esp_err_t audio_pwm_stream_write(void *user,
                                        solar_os_stream_handle_t *handle,
                                        const void *data,
                                        size_t len,
                                        uint32_t timeout_ms,
                                        size_t *written)
{
    (void)timeout_ms;
    solar_os_audio_pwm_device_t *device = user;
    if (device == NULL || handle == NULL || handle->context != device ||
        data == NULL || written == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const size_t frame_bytes = handle->audio.channels * sizeof(int16_t);
    if (frame_bytes == 0U || len == 0U || (len % frame_bytes) != 0U) {
        return ESP_ERR_INVALID_SIZE;
    }
    const esp_err_t err = audio_pwm_write_s16(data,
                                              len / frame_bytes,
                                              handle->audio.channels,
                                              device->volume);
    if (err == ESP_OK) {
        *written = len;
    }
    return err;
}

static esp_err_t audio_pwm_set_volume(void *user, uint8_t volume)
{
    solar_os_audio_pwm_device_t *device = user;
    if (device == NULL || !device->attached || volume > 100U) {
        return ESP_ERR_INVALID_ARG;
    }
    device->volume = volume;
    return ESP_OK;
}

static esp_err_t audio_pwm_register_stream(solar_os_audio_pwm_device_t *device)
{
    solar_os_stream_driver_t driver = {
        .info = {
            .type = SOLAR_OS_STREAM_TYPE_AUDIO,
            .direction = SOLAR_OS_STREAM_DIRECTION_SINK,
            .sharing = SOLAR_OS_STREAM_SHARING_EXCLUSIVE,
            .audio = device->native_format,
        },
        .open = audio_pwm_stream_open,
        .close = audio_pwm_stream_close,
        .write = audio_pwm_stream_write,
        .user = device,
    };
    strlcpy(driver.info.id, device->stream_id, sizeof(driver.info.id));
    strlcpy(driver.info.provider, "audio-pwm", sizeof(driver.info.provider));
    strlcpy(driver.info.device, device->id, sizeof(driver.info.device));
    strlcpy(driver.info.unit, "frames", sizeof(driver.info.unit));
    strlcpy(driver.info.format, "pcm-s16le", sizeof(driver.info.format));
    strlcpy(driver.info.summary,
            device->pin_right >= 0 ? "LEDC PWM stereo audio playback" :
                                     "LEDC PWM audio playback",
            sizeof(driver.info.summary));
    return solar_os_stream_register(&driver);
}

esp_err_t solar_os_audio_pwm_attach(
    const char *name,
    const solar_os_expansion_binding_t *bindings,
    size_t binding_count)
{
    if (name == NULL || bindings == NULL || binding_count == 0U ||
        strnlen(name, AUDIO_PWM_DEVICE_NAME_MAX) >= AUDIO_PWM_DEVICE_NAME_MAX ||
        pwm_audio.attached) {
        return pwm_audio.attached ? ESP_ERR_NOT_ALLOWED : ESP_ERR_INVALID_ARG;
    }

    int pin = -1;
    int pin_right = -1;
    for (size_t i = 0; i < binding_count; i++) {
        if (bindings[i].kind != SOLAR_OS_EXPANSION_BINDING_PWM) {
            continue;
        }
        if (strcmp(bindings[i].role, "pwm") == 0 && pin < 0) {
            pin = bindings[i].value;
        } else if (strcmp(bindings[i].role, "pwm_right") == 0 && pin_right < 0) {
            pin_right = bindings[i].value;
        }
    }
    if (pin < 0 || (pin_right >= 0 && pin_right == pin)) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Drive the outputs low and hold them there between sounds rather than
     * leave them floating. Failures from here on release the hold. */
    audio_pwm_park_pins((gpio_num_t)pin, (gpio_num_t)pin_right);

    memset(&pwm_audio, 0, sizeof(pwm_audio));
    pwm_audio.pin = pin;
    pwm_audio.pin_right = pin_right;
    pwm_audio.native_format = pwm_audio_native_format_for(pin_right >= 0);
    pwm_audio.volume = 70U;
    strlcpy(pwm_audio.id, name, sizeof(pwm_audio.id));
    const int stream_len = snprintf(pwm_audio.stream_id,
                                    sizeof(pwm_audio.stream_id),
                                    "%s.playback",
                                    name);
    if (stream_len < 0 || (size_t)stream_len >= sizeof(pwm_audio.stream_id)) {
        memset(&pwm_audio, 0, sizeof(pwm_audio));
        audio_pwm_release_pins((gpio_num_t)pin, (gpio_num_t)pin_right);
        return ESP_ERR_INVALID_ARG;
    }
    pwm_audio.attached = true;

    esp_err_t err = audio_pwm_register_stream(&pwm_audio);
    if (err != ESP_OK) {
        memset(&pwm_audio, 0, sizeof(pwm_audio));
        audio_pwm_release_pins((gpio_num_t)pin, (gpio_num_t)pin_right);
        return err;
    }

    solar_os_audio_device_info_t info = {
        .capabilities = SOLAR_OS_AUDIO_DEVICE_CAP_OUTPUT |
                        SOLAR_OS_AUDIO_DEVICE_CAP_VOLUME,
        .native_format = pwm_audio.native_format,
    };
    strlcpy(info.id, pwm_audio.id, sizeof(info.id));
    if (pin_right >= 0) {
        snprintf(info.name, sizeof(info.name), "LEDC PWM GPIO%d/%d", pin, pin_right);
    } else {
        snprintf(info.name, sizeof(info.name), "LEDC PWM on GPIO%d", pin);
    }
    strlcpy(info.provider, "expansion", sizeof(info.provider));
    strlcpy(info.playback_stream,
            pwm_audio.stream_id,
            sizeof(info.playback_stream));
    const solar_os_audio_device_ops_t ops = {
        .set_volume = audio_pwm_set_volume,
    };
    err = solar_os_audio_register_device_ex(&info, &ops, &pwm_audio);
    if (err != ESP_OK) {
        (void)solar_os_stream_unregister(pwm_audio.stream_id);
        memset(&pwm_audio, 0, sizeof(pwm_audio));
        audio_pwm_release_pins((gpio_num_t)pin, (gpio_num_t)pin_right);
        return err;
    }
    ESP_LOGI(TAG, "%s attached: %s, held low between sounds", pwm_audio.id, info.name);
    return ESP_OK;
}

esp_err_t solar_os_audio_pwm_detach(const char *name)
{
    if (name == NULL || !pwm_audio.attached || strcmp(name, pwm_audio.id) != 0) {
        return ESP_ERR_NOT_FOUND;
    }
    esp_err_t err = solar_os_stream_unregister(pwm_audio.stream_id);
    if (err != ESP_OK) {
        return err;
    }
    err = solar_os_audio_unregister_device(pwm_audio.id);
    if (err != ESP_OK) {
        return err;
    }
    /* Leave the pins driven low but free for their next user. */
    audio_pwm_release_pins((gpio_num_t)pwm_audio.pin, (gpio_num_t)pwm_audio.pin_right);
    memset(&pwm_audio, 0, sizeof(pwm_audio));
    return ESP_OK;
}
