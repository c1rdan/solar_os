#include "solar_os_psram_disk.h"

#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "solar_os_disk_image.h"
#include "solar_os_memory.h"
#include "spi_psram.h"

#define PSRAM_DISK_TYPE_NAME "psram"
#define PSRAM_DISK_NAME_MAX 12U
#define PSRAM_DISK_WORK_BYTES SOLAR_OS_DISK_IMAGE_FORMAT_WORK_BYTES
#define PSRAM_DISK_MARKER_BYTES 16U
/* Pattern spans cross a page boundary, so the low address bits and the burst
 * splitting are exercised too. */
#define PSRAM_DISK_SPAN_BYTES 1536U
#define PSRAM_DISK_SPEED_BYTES (64U * 1024U)
#define PSRAM_DISK_SPEED_OFFSET (1024U * 1024U)

typedef struct {
    bool attached;
    char name[PSRAM_DISK_NAME_MAX];
    spi_psram_t *psram;
} psram_disk_t;

static const char *TAG = "psram-disk";
static psram_disk_t disk;

static esp_err_t device_read(void *context, uint32_t offset, void *data, size_t length)
{
    return spi_psram_read(context, offset, data, length);
}

static esp_err_t device_write(void *context, uint32_t offset, const void *data, size_t length)
{
    return spi_psram_write(context, offset, data, length);
}

static const solar_os_disk_image_device_t psram_device = {
    .read = device_read,
    .write = device_write,
};

static uint8_t pattern_byte(uint32_t seed, size_t index)
{
    uint32_t x = seed ^ (uint32_t)(index / 4U) * 0x9e3779b9U;
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return (uint8_t)(x >> ((index % 4U) * 8U));
}

static void fill_pattern(uint8_t *data, size_t length, uint32_t seed)
{
    for (size_t i = 0; i < length; i++) {
        data[i] = pattern_byte(seed, i);
    }
}

static bool pattern_matches(const uint8_t *data, size_t length, uint32_t seed, size_t *first_bad)
{
    for (size_t i = 0; i < length; i++) {
        if (data[i] != pattern_byte(seed, i)) {
            *first_bad = i;
            return false;
        }
    }
    return true;
}

/* A distinct marker at address 0 and at every power of two from 1 KiB up: a
 * stuck or swapped address line makes two markers share the same cells. */
static esp_err_t check_address_lines(spi_psram_t *psram, uint8_t *buffer)
{
    const uint32_t size = spi_psram_size(psram);
    uint32_t addresses[32];
    unsigned count = 0;
    addresses[count++] = 0U;
    for (uint32_t address = 1024U; address < size && count < 32U; address <<= 1) {
        addresses[count++] = address;
    }
    for (unsigned i = 0; i < count; i++) {
        fill_pattern(buffer, PSRAM_DISK_MARKER_BYTES, 0xa5a50000U + i);
        const esp_err_t err = spi_psram_write(psram, addresses[i], buffer, PSRAM_DISK_MARKER_BYTES);
        if (err != ESP_OK) return err;
    }
    for (unsigned i = 0; i < count; i++) {
        const esp_err_t err = spi_psram_read(psram, addresses[i], buffer, PSRAM_DISK_MARKER_BYTES);
        if (err != ESP_OK) return err;
        size_t bad = 0;
        if (!pattern_matches(buffer, PSRAM_DISK_MARKER_BYTES, 0xa5a50000U + i, &bad)) {
            ESP_LOGE(TAG, "address check failed at 0x%06lx (byte %u)",
                     (unsigned long)addresses[i], (unsigned)bad);
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    return ESP_OK;
}

static esp_err_t check_span(spi_psram_t *psram, uint8_t *buffer, uint32_t address, uint32_t seed)
{
    fill_pattern(buffer, PSRAM_DISK_SPAN_BYTES, seed);
    esp_err_t err = spi_psram_write(psram, address, buffer, PSRAM_DISK_SPAN_BYTES);
    if (err == ESP_OK) {
        memset(buffer, 0, PSRAM_DISK_SPAN_BYTES);
        err = spi_psram_read(psram, address, buffer, PSRAM_DISK_SPAN_BYTES);
    }
    size_t bad = 0;
    if (err == ESP_OK && !pattern_matches(buffer, PSRAM_DISK_SPAN_BYTES, seed, &bad)) {
        ESP_LOGE(TAG, "data check failed at 0x%06lx: read %02x, expected %02x",
                 (unsigned long)(address + bad), buffer[bad], pattern_byte(seed, bad));
        err = ESP_ERR_INVALID_RESPONSE;
    }
    return err;
}

static unsigned kib_per_second(size_t bytes, int64_t microseconds)
{
    return microseconds > 0 ? (unsigned)((uint64_t)bytes * 1000000ULL / (uint64_t)microseconds / 1024ULL)
                            : 0U;
}

/* Times a 64 KiB write and a verified read back. */
static esp_err_t measure(spi_psram_t *psram, uint8_t *buffer, unsigned *write_rate, unsigned *read_rate)
{
    const size_t chunks = PSRAM_DISK_SPEED_BYTES / PSRAM_DISK_WORK_BYTES;
    /* Only the transfers are timed, not making or checking the pattern. */
    int64_t write_us = 0;
    for (size_t chunk = 0; chunk < chunks; chunk++) {
        fill_pattern(buffer, PSRAM_DISK_WORK_BYTES, 0x5eed0000U + (uint32_t)chunk);
        const int64_t write_start = esp_timer_get_time();
        const esp_err_t err = spi_psram_write(psram, PSRAM_DISK_SPEED_OFFSET + chunk * PSRAM_DISK_WORK_BYTES,
                                              buffer, PSRAM_DISK_WORK_BYTES);
        write_us += esp_timer_get_time() - write_start;
        if (err != ESP_OK) return err;
    }
    int64_t read_us = 0;
    for (size_t chunk = 0; chunk < chunks; chunk++) {
        const int64_t read_start = esp_timer_get_time();
        const esp_err_t err = spi_psram_read(psram, PSRAM_DISK_SPEED_OFFSET + chunk * PSRAM_DISK_WORK_BYTES,
                                             buffer, PSRAM_DISK_WORK_BYTES);
        read_us += esp_timer_get_time() - read_start;
        if (err != ESP_OK) return err;
        size_t bad = 0;
        if (!pattern_matches(buffer, PSRAM_DISK_WORK_BYTES, 0x5eed0000U + (uint32_t)chunk, &bad)) {
            ESP_LOGE(TAG, "data check failed at 0x%06lx",
                     (unsigned long)(PSRAM_DISK_SPEED_OFFSET + chunk * PSRAM_DISK_WORK_BYTES + bad));
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    *write_rate = kib_per_second(PSRAM_DISK_SPEED_BYTES, write_us);
    *read_rate = kib_per_second(PSRAM_DISK_SPEED_BYTES, read_us);
    return ESP_OK;
}

static esp_err_t self_test(spi_psram_t *psram, uint8_t *buffer, unsigned *write_rate, unsigned *read_rate)
{
    const uint32_t size = spi_psram_size(psram);
    esp_err_t err = check_address_lines(psram, buffer);
    if (err == ESP_OK) err = check_span(psram, buffer, 7U * SPI_PSRAM_PAGE_BYTES - 512U, 0x0badf00dU);
    if (err == ESP_OK) err = check_span(psram, buffer, size - PSRAM_DISK_SPAN_BYTES, 0xfeedbeefU);
    if (err == ESP_OK) err = measure(psram, buffer, write_rate, read_rate);
    return err;
}

static esp_err_t parse_bindings(const solar_os_expansion_binding_t *bindings, size_t binding_count,
                                spi_psram_config_t *config)
{
    static const char *const pin_roles[] = {"sclk", "cs", "io0", "io1", "io2", "io3"};
    int pins[6] = {-1, -1, -1, -1, -1, -1};
    int host = 3;
    int speed_mhz = (int)(SPI_PSRAM_CLOCK_DEFAULT_HZ / 1000000U);
    for (size_t i = 0; i < binding_count; i++) {
        const solar_os_expansion_binding_t *binding = &bindings[i];
        if (binding->kind == SOLAR_OS_EXPANSION_BINDING_GPIO) {
            for (unsigned role = 0; role < 6U; role++) {
                if (strcmp(binding->role, pin_roles[role]) == 0) pins[role] = binding->value;
            }
        } else if (binding->kind == SOLAR_OS_EXPANSION_BINDING_PARAMETER) {
            if (strcmp(binding->role, "speed") == 0) speed_mhz = binding->value;
            if (strcmp(binding->role, "host") == 0) host = binding->value;
        }
    }
    for (unsigned role = 0; role < 6U; role++) {
        if (pins[role] < 0) {
            ESP_LOGE(TAG, "missing %s binding", pin_roles[role]);
            return ESP_ERR_INVALID_ARG;
        }
    }
    if (host != 2 && host != 3) return ESP_ERR_INVALID_ARG;
    memset(config, 0, sizeof(*config));
    config->host = host == 2 ? SPI2_HOST : SPI3_HOST;
    config->sclk_pin = pins[0];
    config->cs_pin = pins[1];
    for (unsigned line = 0; line < 4U; line++) config->io_pins[line] = pins[2 + line];
    config->clock_hz = (uint32_t)speed_mhz * 1000000U;
    return ESP_OK;
}

esp_err_t solar_os_psram_disk_attach(const char *name,
                                     const solar_os_expansion_binding_t *bindings,
                                     size_t binding_count)
{
    if (name == NULL || bindings == NULL) return ESP_ERR_INVALID_ARG;
    /* One chip owns one SPI host and the single /psram mount point. */
    if (disk.attached) return ESP_ERR_INVALID_STATE;
    spi_psram_config_t config;
    esp_err_t err = parse_bindings(bindings, binding_count, &config);
    if (err != ESP_OK) return err;

    spi_psram_t *psram = NULL;
    err = spi_psram_open(&config, &psram);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s: no PSRAM chip available: %s", name, esp_err_to_name(err));
        return err;
    }
    uint8_t *work = solar_os_memory_alloc(PSRAM_DISK_WORK_BYTES, SOLAR_OS_MEMORY_EXTERNAL_PREFERRED,
                                          "psram-disk");
    if (work == NULL) {
        spi_psram_close(psram);
        return ESP_ERR_NO_MEM;
    }
    unsigned write_rate = 0;
    unsigned read_rate = 0;
    err = self_test(psram, work, &write_rate, &read_rate);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: self-test failed at %lu MHz: %s", name,
                 (unsigned long)(config.clock_hz / 1000000U), esp_err_to_name(err));
    } else {
        err = solar_os_disk_image_attach_device(name, PSRAM_DISK_TYPE_NAME, &psram_device, psram,
                                                spi_psram_size(psram), SOLAR_OS_PSRAM_DISK_MOUNT_POINT,
                                                work, PSRAM_DISK_WORK_BYTES);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s: mounting %s failed: %s", name, SOLAR_OS_PSRAM_DISK_MOUNT_POINT,
                     esp_err_to_name(err));
        }
    }
    solar_os_memory_free(work);
    if (err != ESP_OK) {
        spi_psram_close(psram);
        return err;
    }
    disk.attached = true;
    disk.psram = psram;
    strlcpy(disk.name, name, sizeof(disk.name));
    ESP_LOGI(TAG, "%s: %lu KiB RAM disk at %s (%lu MHz, write %u KiB/s, read %u KiB/s)", name,
             (unsigned long)(spi_psram_size(psram) / 1024U), SOLAR_OS_PSRAM_DISK_MOUNT_POINT,
             (unsigned long)(config.clock_hz / 1000000U), write_rate, read_rate);
    return ESP_OK;
}

esp_err_t solar_os_psram_disk_detach(const char *name)
{
    if (name == NULL || !disk.attached || strcmp(name, disk.name) != 0) return ESP_ERR_NOT_FOUND;
    /* The volume may already be gone, for example after the shutdown unmount. */
    const esp_err_t err = solar_os_disk_image_detach_device(SOLAR_OS_PSRAM_DISK_MOUNT_POINT);
    if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) return err;
    spi_psram_close(disk.psram);
    memset(&disk, 0, sizeof(disk));
    return ESP_OK;
}
