#include "solar_os_picocalc_core.h"

#include <stdbool.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "solar_os_backlight.h"
#include "solar_os_battery.h"
#include "solar_os_board_caps.h"
#include "solar_os_buses.h"
#include "solar_os_input.h"
#include "solar_os_picocalc_codec.h"
#include "solar_os_power.h"
#include "solar_os_shutdown.h"
#include "solar_os_task.h"

#if SOLAR_OS_BOARD_HAS_DISPLAY
#include "solar_os_board_display.h"
#endif

/* Idle wait between FIFO polls. Each poll already costs the register gap plus
 * two I2C transfers at 10 kHz, so this only paces an empty queue. */
#define PICOCALC_CORE_IDLE_MS 10U
#define PICOCALC_CORE_DRAIN_MAX 16U
#define PICOCALC_CORE_TASK_STACK 3072U
#define PICOCALC_CORE_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
/* The controller refreshes its battery register about every 20 s. */
#define PICOCALC_CORE_BATTERY_REFRESH_MS 5000U
/* The controller resets its own I2C peripheral if it hears nothing for 2.5 s,
 * so a failed poll is retried quickly rather than waiting for the next tick. */
#define PICOCALC_CORE_RETRY_MS 100U
#define PICOCALC_CORE_FAULT_ERRORS 3U
#define PICOCALC_CORE_BUS_LOCK_MS 500U
#define PICOCALC_CORE_KEYBOARD_NAME "keyboard0"
#define PICOCALC_CORE_BATTERY_NAME "battery0"
/* The controller drops the whole system supply after this many seconds; it
 * enforces a minimum of 6, so ask for exactly that. */
#define PICOCALC_CORE_POWER_OFF_SECONDS 6U

typedef struct {
    bool active;
    volatile bool stop_requested;
    volatile bool worker_done;
    char name[SOLAR_OS_EXPANSION_DEVICE_NAME_MAX];
    char i2c_bus[SOLAR_OS_EXPANSION_TARGET_MAX];
    uint8_t address;
    uint8_t gap_ms;
    solar_os_input_source_t input_source;
    solar_os_picocalc_keyboard_t keyboard;
    TaskHandle_t worker_task;
    /* One request/response exchange spans a write, a wait and a read, and the
     * controller answers whatever was written last, so exchanges from the
     * worker, the backlight callbacks and power-off must never interleave. */
    StaticSemaphore_t bus_mutex_storage;
    SemaphoreHandle_t bus_mutex;
    /* Worker-only state. */
    unsigned consecutive_errors;
    bool responding;
    /* Cached battery reading, shared with the battery provider callback. */
    portMUX_TYPE battery_lock;
    solar_os_picocalc_battery_t battery;
    uint32_t transitions;
    uint32_t unsupported;
    uint32_t dropped;
    uint32_t bus_errors;
    /* Replies that answer some other register. */
    uint32_t protocol_errors;
    /* Reads repeated after a failed read. */
    uint32_t read_retries;
} picocalc_core_device_t;

static const char *TAG = "picocalc-core";
static picocalc_core_device_t core;

static esp_err_t parse_bindings(const solar_os_expansion_binding_t *bindings,
                                size_t binding_count,
                                char *i2c_bus,
                                size_t i2c_bus_len,
                                uint8_t *address,
                                uint8_t *gap_ms)
{
    bool have_i2c = false;
    bool have_address = false;

    if (bindings == NULL || i2c_bus == NULL || address == NULL || gap_ms == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    i2c_bus[0] = '\0';
    *address = 0U;
    *gap_ms = SOLAR_OS_PICOCALC_CORE_DEFAULT_GAP_MS;

    for (size_t i = 0; i < binding_count; i++) {
        const solar_os_expansion_binding_t *binding = &bindings[i];
        switch (binding->kind) {
        case SOLAR_OS_EXPANSION_BINDING_I2C_BUS:
            if (have_i2c) {
                return ESP_ERR_INVALID_ARG;
            }
            strlcpy(i2c_bus, binding->target, i2c_bus_len);
            have_i2c = true;
            break;
        case SOLAR_OS_EXPANSION_BINDING_I2C_ADDRESS:
            if (have_address || binding->value != SOLAR_OS_PICOCALC_CORE_ADDRESS) {
                return ESP_ERR_INVALID_ARG;
            }
            *address = (uint8_t)binding->value;
            have_address = true;
            break;
        case SOLAR_OS_EXPANSION_BINDING_PARAMETER:
            if (strcmp(binding->role, "gap") != 0 || binding->value < 0 ||
                binding->value > SOLAR_OS_PICOCALC_CORE_MAX_GAP_MS) {
                return ESP_ERR_INVALID_ARG;
            }
            *gap_ms = (uint8_t)binding->value;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
        }
    }

    return have_i2c && have_address &&
            solar_os_expansion_find_i2c_bus(i2c_bus, NULL, NULL)
        ? ESP_OK
        : ESP_ERR_INVALID_ARG;
}

static esp_err_t lock_bus(picocalc_core_device_t *device)
{
    return xSemaphoreTake(device->bus_mutex, pdMS_TO_TICKS(PICOCALC_CORE_BUS_LOCK_MS)) == pdTRUE ?
        ESP_OK : ESP_ERR_TIMEOUT;
}

/* Read two bytes from a register: write the register number, wait, read. */
static esp_err_t read_register(picocalc_core_device_t *device,
                               uint8_t reg,
                               uint8_t data[2])
{
    ESP_RETURN_ON_ERROR(lock_bus(device), TAG, "bus busy");
    esp_err_t err = solar_os_bus_i2c_transmit(device->i2c_bus, device->address, &reg, 1U);
    if (err == ESP_OK) {
        if (device->gap_ms > 0U) {
            vTaskDelay(pdMS_TO_TICKS(device->gap_ms));
        }
        err = solar_os_bus_i2c_receive(device->i2c_bus, device->address, data, 2U);
        if (err != ESP_OK) {
            /* The controller keeps its reply until the next write, so read it
             * again without writing: a second FIFO write would dequeue the
             * next entry and lose this one. */
            device->read_retries++;
            err = solar_os_bus_i2c_receive(device->i2c_bus, device->address, data, 2U);
        }
    }
    xSemaphoreGive(device->bus_mutex);
    return err;
}

static esp_err_t write_register(picocalc_core_device_t *device, uint8_t reg, uint8_t value)
{
    const uint8_t frame[2] = {(uint8_t)(reg | SOLAR_OS_PICOCALC_REG_WRITE), value};
    ESP_RETURN_ON_ERROR(lock_bus(device), TAG, "bus busy");
    const esp_err_t err = solar_os_bus_i2c_transmit(device->i2c_bus, device->address,
                                                    frame, sizeof(frame));
    xSemaphoreGive(device->bus_mutex);
    return err;
}

static void set_responding(picocalc_core_device_t *device, bool responding)
{
    if (device->responding == responding) {
        return;
    }
    device->responding = responding;
    (void)solar_os_input_keyboard_source_set_ready(device->input_source, responding);
    if (responding) {
        ESP_LOGI(TAG, "%s responding on %s", device->name, device->i2c_bus);
    } else {
        ESP_LOGW(TAG, "%s stopped responding on %s: is the PicoCalc powered?",
                 device->name, device->i2c_bus);
    }
}

static void release_all_keys(picocalc_core_device_t *device)
{
    solar_os_picocalc_keyboard_reset(&device->keyboard);
    solar_os_input_source_release_all(device->input_source);
}

static void publish_entry(picocalc_core_device_t *device, uint8_t state, uint8_t code)
{
    solar_os_picocalc_key_transition_t transition;
    switch (solar_os_picocalc_keyboard_decode(&device->keyboard, state, code, &transition)) {
    case SOLAR_OS_PICOCALC_KEY_TRANSITION: {
        const esp_err_t err = solar_os_input_write_key(
            device->input_source,
            transition.physical_key,
            SOLAR_OS_INPUT_USAGE_NONE,
            transition.key,
            transition.modifiers,
            transition.pressed ? SOLAR_OS_INPUT_KEY_PRESS : SOLAR_OS_INPUT_KEY_RELEASE);
        if (err == ESP_OK) {
            device->transitions++;
        } else {
            device->dropped++;
        }
        break;
    }
    case SOLAR_OS_PICOCALC_KEY_UNSUPPORTED:
        device->unsupported++;
        break;
    case SOLAR_OS_PICOCALC_KEY_POWER: {
        /* The mainboard power key. A long press is handled by the controller
         * itself and cuts power outright; a short press asks for an orderly
         * shutdown. */
        const esp_err_t err = solar_os_shutdown_request();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "power key shutdown request failed: %s", esp_err_to_name(err));
        }
        break;
    }
    case SOLAR_OS_PICOCALC_KEY_NONE:
    default:
        break;
    }
}

static void refresh_battery(picocalc_core_device_t *device)
{
    uint8_t data[2] = {0};
    if (read_register(device, SOLAR_OS_PICOCALC_REG_BATTERY, data) != ESP_OK) {
        return;
    }
    solar_os_picocalc_battery_t battery = {0};
    if (!solar_os_picocalc_decode_battery(data, &battery)) {
        /* Not a battery reply: keep the last reading. */
        device->protocol_errors++;
        return;
    }
    portENTER_CRITICAL(&device->battery_lock);
    device->battery = battery;
    portEXIT_CRITICAL(&device->battery_lock);
}

static esp_err_t battery_read(void *user, solar_os_battery_sample_t *sample)
{
    picocalc_core_device_t *device = user;
    if (device == NULL || !device->active || sample == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    portENTER_CRITICAL(&device->battery_lock);
    const solar_os_picocalc_battery_t battery = device->battery;
    portEXIT_CRITICAL(&device->battery_lock);

    /* The controller reports 0 when there is no battery or no reading yet. */
    if (!battery.valid) {
        return ESP_ERR_NOT_FOUND;
    }

    /* The controller gives a percentage and a charging flag but no voltage.
     * Report an estimate from the configured empty..full range so that
     * voltage-based consumers agree with the percentage; it is not measured,
     * hence not calibrated. */
    solar_os_battery_config_t config;
    solar_os_battery_get_config(&config);
    *sample = (solar_os_battery_sample_t) {
        .battery_mv = solar_os_picocalc_estimate_millivolts(battery.percent,
                                                            config.min_voltage_mv,
                                                            config.max_voltage_mv),
        .calibrated = false,
        .voltage_estimated = true,
        .percent_valid = true,
        .percent = battery.percent,
        /* Charging implies external power; the converse is unknown. */
        .external_power_valid = battery.charging,
        .external_power = battery.charging,
        .charging_valid = true,
        .charging = battery.charging,
    };
    return ESP_OK;
}

static void log_version(picocalc_core_device_t *device)
{
    uint8_t data[2] = {0};
    if (read_register(device, SOLAR_OS_PICOCALC_REG_VERSION, data) == ESP_OK) {
        ESP_LOGI(TAG, "keyboard controller firmware 0x%02x", data[1]);
    }
}

static void worker(void *arg)
{
    picocalc_core_device_t *device = arg;
    TickType_t last_battery = xTaskGetTickCount() - pdMS_TO_TICKS(PICOCALC_CORE_BATTERY_REFRESH_MS);

    while (!device->stop_requested) {
        bool activity = false;
        esp_err_t err = ESP_OK;

        for (unsigned drained = 0U; drained < PICOCALC_CORE_DRAIN_MAX; drained++) {
            uint8_t entry[2] = {0};
            err = read_register(device, SOLAR_OS_PICOCALC_REG_FIFO, entry);
            if (err != ESP_OK) {
                break;
            }
            if (!solar_os_picocalc_keyboard_fifo_valid(entry[0], entry[1])) {
                /* A reply to some other register: count it and leave the rest
                 * of the queue to the next poll. */
                device->protocol_errors++;
                break;
            }
            if (solar_os_picocalc_keyboard_fifo_empty(entry[0], entry[1])) {
                break;
            }
            activity = true;
            publish_entry(device, entry[0], entry[1]);
        }

        if (err == ESP_OK) {
            device->consecutive_errors = 0U;
            if (!device->responding) {
                set_responding(device, true);
                log_version(device);
            }
            if (xTaskGetTickCount() - last_battery >=
                pdMS_TO_TICKS(PICOCALC_CORE_BATTERY_REFRESH_MS)) {
                refresh_battery(device);
                last_battery = xTaskGetTickCount();
            }
        } else {
            device->bus_errors++;
            if (++device->consecutive_errors == PICOCALC_CORE_FAULT_ERRORS) {
                /* No key may stay held, or it would repeat forever. */
                release_all_keys(device);
                set_responding(device, false);
            }
            vTaskDelay(pdMS_TO_TICKS(PICOCALC_CORE_RETRY_MS));
        }

        if (!activity && err == ESP_OK) {
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(PICOCALC_CORE_IDLE_MS));
        }
    }

    device->worker_done = true;
    solar_os_task_delete_internal(NULL);
}

static esp_err_t keyboard_backlight(void *user, uint8_t percent)
{
    picocalc_core_device_t *device = user;
    return write_register(device,
                          SOLAR_OS_PICOCALC_REG_KBD_BACKLIGHT,
                          solar_os_picocalc_kbd_register_from_percent(percent));
}

#if SOLAR_OS_BOARD_HAS_DISPLAY
static esp_err_t lcd_backlight(void *user, uint8_t percent)
{
    picocalc_core_device_t *device = user;
    return write_register(device,
                          SOLAR_OS_PICOCALC_REG_LCD_BACKLIGHT,
                          solar_os_picocalc_lcd_register_from_percent(percent));
}
#endif

static esp_err_t power_off(void *user)
{
    picocalc_core_device_t *device = user;
    if (device == NULL || !device->active) {
        return ESP_ERR_INVALID_STATE;
    }
    /* The controller waits this long and then removes power from the whole
     * system, including this chip. */
    return write_register(device, SOLAR_OS_PICOCALC_REG_POWER_OFF, PICOCALC_CORE_POWER_OFF_SECONDS);
}

static void clear_device(picocalc_core_device_t *device)
{
    solar_os_power_unregister_off_provider(device);
#if SOLAR_OS_BOARD_HAS_DISPLAY
    solar_os_board_display_clear_brightness_provider(device);
#endif
    solar_os_backlight_unregister(device);
    if (device->input_source != SOLAR_OS_INPUT_SOURCE_INVALID) {
        solar_os_input_source_close(device->input_source);
    }
    memset(device, 0, sizeof(*device));
}

esp_err_t solar_os_picocalc_core_attach(
    const char *name,
    const solar_os_expansion_binding_t *bindings,
    size_t binding_count)
{
    char i2c_bus[SOLAR_OS_EXPANSION_TARGET_MAX] = {0};
    uint8_t address = 0U;
    uint8_t gap_ms = 0U;

    if (name == NULL || name[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    if (core.active) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_RETURN_ON_ERROR(parse_bindings(bindings, binding_count, i2c_bus, sizeof(i2c_bus),
                                       &address, &gap_ms),
                        TAG, "invalid bindings");

    memset(&core, 0, sizeof(core));
    core.active = true;
    core.address = address;
    core.gap_ms = gap_ms;
    strlcpy(core.name, name, sizeof(core.name));
    strlcpy(core.i2c_bus, i2c_bus, sizeof(core.i2c_bus));
    core.bus_mutex = xSemaphoreCreateMutexStatic(&core.bus_mutex_storage);
    portMUX_INITIALIZE(&core.battery_lock);
    solar_os_picocalc_keyboard_reset(&core.keyboard);

    /* Not finding the controller is not fatal: it only runs on PicoCalc power,
     * and polling picks it up as soon as it answers. */
    const esp_err_t probe = solar_os_bus_i2c_probe(i2c_bus, address);
    if (probe != ESP_OK) {
        ESP_LOGW(TAG, "controller not found at 0x%02x on %s (%s); will keep polling",
                 address, i2c_bus, esp_err_to_name(probe));
    }

    esp_err_t err = solar_os_input_keyboard_source_open(PICOCALC_CORE_KEYBOARD_NAME,
                                                        false, &core.input_source);
    if (err != ESP_OK) {
        clear_device(&core);
        return err;
    }
    err = solar_os_backlight_register(keyboard_backlight, &core);
    if (err != ESP_OK) {
        clear_device(&core);
        return err;
    }
    const solar_os_battery_provider_t battery_provider = {
        .read = battery_read,
        .user = &core,
    };
    err = solar_os_battery_register_provider(PICOCALC_CORE_BATTERY_NAME, &battery_provider);
    if (err != ESP_OK) {
        clear_device(&core);
        return err;
    }
    if (solar_os_task_create_pinned_internal(worker,
                                             core.name,
                                             PICOCALC_CORE_TASK_STACK,
                                             &core,
                                             PICOCALC_CORE_TASK_PRIORITY,
                                             &core.worker_task,
                                             tskNO_AFFINITY,
                                             SOLAR_OS_TASK_ROLE_BACKGROUND) != pdPASS) {
        (void)solar_os_battery_unregister_provider(PICOCALC_CORE_BATTERY_NAME);
        clear_device(&core);
        return ESP_ERR_NO_MEM;
    }

    (void)solar_os_power_register_off_provider(name, power_off, &core);
#if SOLAR_OS_BOARD_HAS_DISPLAY
    /* The panel backlight is driven by this controller, not by a GPIO. Binding
     * applies the brightness the display service has already requested. */
    err = solar_os_board_display_set_brightness_provider(lcd_backlight, &core);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LCD backlight not bound: %s", esp_err_to_name(err));
    }
#endif
    ESP_LOGI(TAG, "%s attached on %s address 0x%02x (register gap %u ms) as %s and %s",
             name, i2c_bus, address, (unsigned)gap_ms,
             PICOCALC_CORE_KEYBOARD_NAME, PICOCALC_CORE_BATTERY_NAME);
    return ESP_OK;
}

esp_err_t solar_os_picocalc_core_detach(const char *name)
{
    if (!core.active || name == NULL || strcmp(core.name, name) != 0) {
        return ESP_ERR_NOT_FOUND;
    }

    core.stop_requested = true;
    if (core.worker_task != NULL) {
        (void)xTaskNotifyGive(core.worker_task);
    }
    if (!solar_os_task_wait_done(core.worker_task, &core.worker_done,
                                 SOLAR_OS_TASK_STOP_WAIT_MS)) {
        return ESP_ERR_TIMEOUT;
    }
    ESP_RETURN_ON_ERROR(solar_os_battery_unregister_provider(PICOCALC_CORE_BATTERY_NAME),
                        TAG, "battery provider unregister failed");

    ESP_LOGI(TAG,
             "%s detached: %lu transitions, %lu unsupported, %lu dropped, %lu bus errors, "
             "%lu protocol errors, %lu read retries",
             name,
             (unsigned long)core.transitions,
             (unsigned long)core.unsupported,
             (unsigned long)core.dropped,
             (unsigned long)core.bus_errors,
             (unsigned long)core.protocol_errors,
             (unsigned long)core.read_retries);
    clear_device(&core);
    return ESP_OK;
}
