#include "solar_os_board_display.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "board-display";

static solar_os_board_display_t *primary_display;
static portMUX_TYPE primary_display_lock = portMUX_INITIALIZER_UNLOCKED;

/* Externally provided brightness (see the header). The mutex serialises the
 * provider callback with binding and unbinding; the spinlock only guards the
 * small shared fields so brightness queries never block on a slow callback. */
static portMUX_TYPE brightness_lock = portMUX_INITIALIZER_UNLOCKED;
static StaticSemaphore_t brightness_mutex_storage;
static SemaphoreHandle_t brightness_mutex;
static solar_os_board_display_brightness_fn_t brightness_provider;
static void *brightness_provider_user;
static uint8_t brightness_requested = 100U;
/* The panel is in power save: a provider is held at its lowest level. */
static bool brightness_power_save;

static SemaphoreHandle_t brightness_mutex_get(void)
{
    portENTER_CRITICAL(&brightness_lock);
    if (brightness_mutex == NULL) {
        brightness_mutex = xSemaphoreCreateMutexStatic(&brightness_mutex_storage);
    }
    SemaphoreHandle_t mutex = brightness_mutex;
    portEXIT_CRITICAL(&brightness_lock);
    return mutex;
}

static bool brightness_provider_bound(void)
{
    portENTER_CRITICAL(&brightness_lock);
    const bool bound = brightness_provider != NULL;
    portEXIT_CRITICAL(&brightness_lock);
    return bound;
}

esp_err_t solar_os_board_display_set_brightness_provider(
    solar_os_board_display_brightness_fn_t set, void *user)
{
    if (set == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    SemaphoreHandle_t mutex = brightness_mutex_get();
    xSemaphoreTake(mutex, portMAX_DELAY);
    portENTER_CRITICAL(&brightness_lock);
    const bool taken = brightness_provider != NULL;
    if (!taken) {
        brightness_provider = set;
        brightness_provider_user = user;
    }
    const uint8_t percent = brightness_power_save ? 0U : brightness_requested;
    portEXIT_CRITICAL(&brightness_lock);
    esp_err_t err = taken ? ESP_ERR_INVALID_STATE : set(user, percent);
    xSemaphoreGive(mutex);
    return err;
}

void solar_os_board_display_clear_brightness_provider(void *user)
{
    SemaphoreHandle_t mutex = brightness_mutex_get();
    xSemaphoreTake(mutex, portMAX_DELAY);
    portENTER_CRITICAL(&brightness_lock);
    if (brightness_provider != NULL && brightness_provider_user == user) {
        brightness_provider = NULL;
        brightness_provider_user = NULL;
    }
    portEXIT_CRITICAL(&brightness_lock);
    xSemaphoreGive(mutex);
}

esp_err_t solar_os_board_display_register_primary(solar_os_board_display_t *display)
{
    if (display == NULL || display->ops == NULL || display->driver == NULL ||
        display->u8g2 == NULL || display->driver_name == NULL ||
        display->controller == NULL || display->width == 0 || display->height == 0 ||
        display->width != u8g2_GetDisplayWidth(display->u8g2) ||
        display->height != u8g2_GetDisplayHeight(display->u8g2)) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&primary_display_lock);
    if (primary_display != NULL) {
        portEXIT_CRITICAL(&primary_display_lock);
        return ESP_ERR_INVALID_STATE;
    }
    primary_display = display;
    portEXIT_CRITICAL(&primary_display_lock);
    return ESP_OK;
}

esp_err_t solar_os_board_display_unregister_primary(solar_os_board_display_t *display)
{
    if (display == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&primary_display_lock);
    if (primary_display != display) {
        portEXIT_CRITICAL(&primary_display_lock);
        return ESP_ERR_NOT_FOUND;
    }
    primary_display = NULL;
    portEXIT_CRITICAL(&primary_display_lock);
    return ESP_OK;
}

esp_err_t solar_os_board_display_init(solar_os_board_display_t *display)
{
    if (display == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&primary_display_lock);
    solar_os_board_display_t *registered = primary_display;
    if (registered != NULL) {
        *display = *registered;
    }
    portEXIT_CRITICAL(&primary_display_lock);
    return registered != NULL ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t solar_os_board_display_runtime_ready(solar_os_board_display_t *display)
{
    return display != NULL && display->ops != NULL && display->ops->runtime_ready != NULL ?
        display->ops->runtime_ready(display) : ESP_ERR_INVALID_STATE;
}

esp_err_t solar_os_board_display_resume(solar_os_board_display_t *display)
{
    return display != NULL && display->ops != NULL && display->ops->resume != NULL ?
        display->ops->resume(display) : ESP_ERR_INVALID_STATE;
}

void solar_os_board_display_deinit(solar_os_board_display_t *display)
{
    if (display != NULL && display->ops != NULL && display->ops->deinit != NULL) {
        display->ops->deinit(display);
    }
}

esp_err_t solar_os_board_display_set_power_save(solar_os_board_display_t *display,
                                               bool enabled)
{
    if (display == NULL || display->u8g2 == NULL || display->ops == NULL)
        return ESP_ERR_INVALID_STATE;
    esp_err_t err = ESP_OK;
    if (display->ops->set_power_save != NULL)
        err = display->ops->set_power_save(display, enabled);
    else
        u8g2_SetPowerSave(display->u8g2, enabled ? 1 : 0);
    if (err != ESP_OK)
        return err;

    /* The panel driver only switches a backlight pin it owns. A provider
     * backlight drops to its lowest level while the panel sleeps and gets the
     * requested level back on wake. Like a pin backlight, a failure here does
     * not fail the panel transition. */
    SemaphoreHandle_t mutex = brightness_mutex_get();
    xSemaphoreTake(mutex, portMAX_DELAY);
    portENTER_CRITICAL(&brightness_lock);
    brightness_power_save = enabled;
    solar_os_board_display_brightness_fn_t provider = brightness_provider;
    void *user = brightness_provider_user;
    const uint8_t percent = enabled ? 0U : brightness_requested;
    portEXIT_CRITICAL(&brightness_lock);
    if (provider != NULL) {
        const esp_err_t backlight_err = provider(user, percent);
        if (backlight_err != ESP_OK)
            ESP_LOGW(TAG, "backlight %s failed: %s", enabled ? "dim" : "restore",
                     esp_err_to_name(backlight_err));
    }
    xSemaphoreGive(mutex);
    return ESP_OK;
}

u8g2_t *solar_os_board_display_u8g2(solar_os_board_display_t *display)
{
    return display != NULL ? display->u8g2 : NULL;
}

const char *solar_os_board_display_driver_name(const solar_os_board_display_t *display)
{
    return display != NULL && display->driver_name != NULL ? display->driver_name : "unknown";
}

const char *solar_os_board_display_controller(const solar_os_board_display_t *display)
{
    return display != NULL && display->controller != NULL ? display->controller : "unknown";
}

uint16_t solar_os_board_display_width(const solar_os_board_display_t *display)
{
    return display != NULL ? display->width : 0;
}

uint16_t solar_os_board_display_height(const solar_os_board_display_t *display)
{
    return display != NULL ? display->height : 0;
}

bool solar_os_board_display_ready(const solar_os_board_display_t *display)
{
    return display != NULL && display->ready;
}

uint32_t solar_os_board_display_surface_formats(
    const solar_os_board_display_t *display)
{
    return display != NULL ? display->surface_formats : 0U;
}

uint32_t solar_os_board_display_frame_formats(
    const solar_os_board_display_t *display)
{
    return display != NULL ? display->frame_formats : 0U;
}

uint16_t solar_os_board_display_preferred_stream_fps(
    const solar_os_board_display_t *display)
{
    return display != NULL ? display->preferred_stream_fps : 0U;
}

uint32_t solar_os_board_display_max_stream_pixels_per_second(
    const solar_os_board_display_t *display)
{
    return display != NULL ? display->max_stream_pixels_per_second : 0U;
}

bool solar_os_board_display_brightness_supported(const solar_os_board_display_t *display)
{
    if (display != NULL && brightness_provider_bound()) {
        return true;
    }
    return display != NULL && display->ops != NULL &&
        display->ops->brightness_supported != NULL &&
        display->ops->brightness_supported(display);
}

esp_err_t solar_os_board_display_get_brightness(const solar_os_board_display_t *display,
                                                uint8_t *percent)
{
    if (percent == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (display != NULL && brightness_provider_bound()) {
        portENTER_CRITICAL(&brightness_lock);
        *percent = brightness_requested;
        portEXIT_CRITICAL(&brightness_lock);
        return ESP_OK;
    }
    return display != NULL && display->ops != NULL && display->ops->get_brightness != NULL ?
        display->ops->get_brightness(display, percent) : ESP_ERR_NOT_SUPPORTED;
}

esp_err_t solar_os_board_display_set_brightness(solar_os_board_display_t *display,
                                                uint8_t percent)
{
    if (display == NULL) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (percent > 100U) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Remember the request even when nothing can honour it yet, so a provider
     * that binds later starts at the level the display service restored. */
    portENTER_CRITICAL(&brightness_lock);
    brightness_requested = percent;
    portEXIT_CRITICAL(&brightness_lock);

    SemaphoreHandle_t mutex = brightness_mutex_get();
    xSemaphoreTake(mutex, portMAX_DELAY);
    portENTER_CRITICAL(&brightness_lock);
    solar_os_board_display_brightness_fn_t provider = brightness_provider;
    void *user = brightness_provider_user;
    const bool power_save = brightness_power_save;
    portEXIT_CRITICAL(&brightness_lock);
    esp_err_t err;
    if (provider != NULL) {
        /* While the panel sleeps the level waits for wake. */
        err = power_save ? ESP_OK : provider(user, percent);
    } else {
        err = display->ops != NULL && display->ops->set_brightness != NULL ?
            display->ops->set_brightness(display, percent) : ESP_ERR_NOT_SUPPORTED;
    }
    xSemaphoreGive(mutex);
    return err;
}

esp_err_t solar_os_board_display_set_colors(solar_os_board_display_t *display,
                                            uint32_t foreground_rgb888,
                                            uint32_t background_rgb888)
{
    return display != NULL && display->ops != NULL && display->ops->set_colors != NULL ?
        display->ops->set_colors(display, foreground_rgb888, background_rgb888) : ESP_OK;
}

const char *solar_os_board_display_controller_mode(const solar_os_board_display_t *display)
{
    return display != NULL && display->ops != NULL && display->ops->controller_mode != NULL ?
        display->ops->controller_mode(display) : NULL;
}

const char *solar_os_board_display_controller_mode_values(const solar_os_board_display_t *display)
{
    return display != NULL && display->ops != NULL &&
        display->ops->controller_mode_values != NULL ?
        display->ops->controller_mode_values(display) : NULL;
}

esp_err_t solar_os_board_display_set_controller_mode(solar_os_board_display_t *display,
                                                     const char *mode)
{
    return display != NULL && display->ops != NULL &&
        display->ops->set_controller_mode != NULL ?
        display->ops->set_controller_mode(display, mode) : ESP_ERR_NOT_SUPPORTED;
}

esp_err_t solar_os_board_display_set_high_refresh_override(
    solar_os_board_display_t *display,
    bool enabled,
    uint16_t hz_tenths)
{
    return display != NULL && display->ops != NULL &&
        display->ops->set_high_refresh_override != NULL ?
        display->ops->set_high_refresh_override(display, enabled, hz_tenths) :
        ESP_ERR_NOT_SUPPORTED;
}

esp_err_t solar_os_board_display_present_mono_xbm(solar_os_board_display_t *display,
                                                  const uint8_t *bitmap,
                                                  size_t bitmap_size,
                                                  uint16_t x,
                                                  uint16_t y,
                                                  uint16_t width,
                                                  uint16_t height,
                                                  uint16_t stride,
                                                  bool palette_inverted)
{
    return display != NULL && display->ops != NULL && display->ops->present_mono_xbm != NULL ?
        display->ops->present_mono_xbm(display,
                                       bitmap,
                                       bitmap_size,
                                       x,
                                       y,
                                       width,
                                       height,
                                       stride,
                                       palette_inverted) : ESP_ERR_NOT_SUPPORTED;
}

esp_err_t solar_os_board_display_present_surface(
    solar_os_board_display_t *display,
    const solar_os_display_surface_t *surface)
{
    return display != NULL && display->ops != NULL &&
        display->ops->present_surface != NULL ?
        display->ops->present_surface(display, surface) : ESP_ERR_NOT_SUPPORTED;
}

esp_err_t solar_os_board_display_present_frame(
    solar_os_board_display_t *display,
    const solar_os_display_raster_t *frame)
{
    return display != NULL && display->ops != NULL &&
        display->ops->present_frame != NULL ?
        display->ops->present_frame(display, frame) : ESP_ERR_NOT_SUPPORTED;
}
