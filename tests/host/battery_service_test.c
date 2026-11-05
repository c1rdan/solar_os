#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nvs.h"
#include "solar_os_battery.h"
#include "solar_os_stream.h"

static solar_os_battery_sample_t provider_sample;
static unsigned stream_register_count;
static unsigned stream_unregister_count;

size_t strlcpy(char *dst, const char *src, size_t size)
{
    const size_t length = strlen(src);
    if (size > 0U) {
        const size_t copy = length < size - 1U ? length : size - 1U;
        memcpy(dst, src, copy);
        dst[copy] = '\0';
    }
    return length;
}

esp_err_t nvs_open(const char *name, nvs_open_mode_t mode, nvs_handle_t *handle)
{
    (void)name;
    if (mode == NVS_READWRITE) {
        *handle = 1;
        return ESP_OK;
    }
    return ESP_ERR_NVS_NOT_FOUND;
}

esp_err_t nvs_get_u16(nvs_handle_t handle, const char *key, uint16_t *value)
{
    (void)handle;
    (void)key;
    (void)value;
    return ESP_ERR_NVS_NOT_FOUND;
}

esp_err_t nvs_get_u32(nvs_handle_t handle, const char *key, uint32_t *value)
{
    (void)handle;
    (void)key;
    (void)value;
    return ESP_ERR_NVS_NOT_FOUND;
}

esp_err_t nvs_set_u16(nvs_handle_t handle, const char *key, uint16_t value)
{
    (void)handle;
    (void)key;
    (void)value;
    return ESP_OK;
}

esp_err_t nvs_set_u32(nvs_handle_t handle, const char *key, uint32_t value)
{
    (void)handle;
    (void)key;
    (void)value;
    return ESP_OK;
}

esp_err_t nvs_commit(nvs_handle_t handle)
{
    (void)handle;
    return ESP_OK;
}

void nvs_close(nvs_handle_t handle)
{
    (void)handle;
}

esp_err_t solar_os_stream_register(const solar_os_stream_driver_t *driver)
{
    assert(driver != NULL);
    assert(strcmp(driver->info.id, "battery") == 0);
    stream_register_count++;
    return ESP_OK;
}

esp_err_t solar_os_stream_unregister(const char *id)
{
    assert(id != NULL);
    assert(strcmp(id, "battery") == 0);
    stream_unregister_count++;
    return ESP_OK;
}

static esp_err_t read_battery(void *user, solar_os_battery_sample_t *sample)
{
    assert(user == &provider_sample);
    assert(sample != NULL);
    *sample = provider_sample;
    return ESP_OK;
}

static solar_os_battery_status_t read_status(void)
{
    solar_os_battery_status_t status;
    assert(solar_os_battery_get_status(&status) == ESP_OK);
    return status;
}

int main(void)
{
    solar_os_battery_config_t initial_config;
    solar_os_battery_get_config(&initial_config);
#ifdef SOLAR_OS_BOARD_DEFAULT_BATTERY_CAPACITY_MAH
    assert(initial_config.capacity_mah == SOLAR_OS_BOARD_DEFAULT_BATTERY_CAPACITY_MAH);
#else
    assert(initial_config.capacity_mah == 0U);
#endif
    assert(solar_os_battery_set_capacity_mah(2500U) == ESP_OK);
    solar_os_battery_get_config(&initial_config);
    assert(initial_config.capacity_mah == 2500U);
    const solar_os_battery_provider_t provider = {
        .read = read_battery,
        .user = &provider_sample,
    };
    assert(solar_os_battery_register_provider("test-battery", &provider) == ESP_OK);
    assert(solar_os_battery_init() == ESP_OK);
    assert(stream_register_count == 1U);

    provider_sample = (solar_os_battery_sample_t) {
        .battery_mv = 3750U,
        .calibrated = true,
        .external_power_valid = true,
        .external_power = true,
        .charging_valid = true,
        .charging = true,
    };
    solar_os_battery_status_t status = read_status();
    assert(status.external_power);
    assert(status.charging);
    assert(status.charging_known);
    assert(status.percent_estimated);

    provider_sample = (solar_os_battery_sample_t) {
        .battery_mv = 3750U,
        .calibrated = true,
        .percent_valid = true,
        .percent = 73U,
    };
    status = read_status();
    assert(status.percent == 73U);
    assert(!status.percent_estimated);
    assert(status.adc_calibrated && !status.voltage_estimated);

    /* A gauge that reports only a percentage derives the voltage from it. */
    provider_sample = (solar_os_battery_sample_t) {
        .battery_mv = 4120U,
        .voltage_estimated = true,
        .percent_valid = true,
        .percent = 100U,
    };
    status = read_status();
    assert(status.percent == 100U && !status.percent_estimated);
    assert(!status.adc_calibrated && status.voltage_estimated);

    provider_sample.percent = 129U;
    status = read_status();
    assert(status.percent == 100U);
    assert(!status.percent_estimated);

    provider_sample = (solar_os_battery_sample_t) {
        .battery_mv = 5000U,
        .calibrated = true,
        .external_power_valid = true,
        .external_power = false,
        .charging_valid = true,
        .charging = false,
    };
    status = read_status();
    assert(!status.external_power);
    assert(!status.charging);
    assert(status.charging_known);

    provider_sample = (solar_os_battery_sample_t) {
        .battery_mv = 5000U,
        .calibrated = true,
    };
    status = read_status();
    assert(!status.external_power);
    assert(!status.charging);
    assert(!status.charging_known);

    assert(solar_os_battery_unregister_provider("test-battery") == ESP_OK);
    assert(stream_unregister_count == 1U);
    assert(!solar_os_battery_has_provider());

    puts("battery service tests: ok");
    return 0;
}
