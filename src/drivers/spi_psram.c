#include "spi_psram.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "solar_os_memory.h"

#define PSRAM_CMD_RESET_ENABLE 0x66U
#define PSRAM_CMD_RESET 0x99U
#define PSRAM_CMD_READ_ID 0x9fU
#define PSRAM_CMD_FAST_QUAD_READ 0xebU
#define PSRAM_CMD_QUAD_WRITE 0x38U
/* Wait cycles between the address and the data of a Fast Quad Read. */
#define PSRAM_QUAD_READ_DUMMY_CLOCKS 6U
#define PSRAM_ID_BYTES 8U
#define PSRAM_ID_MANUFACTURER 0x0dU
#define PSRAM_ID_KNOWN_GOOD_DIE 0x5dU
/* Reset recovery (tRST is 50 us) plus margin. */
#define PSRAM_RESET_DELAY_US 200U
#define PSRAM_MUTEX_WAIT_MS 1000U
/* Polling transfers keep the CPU busy without blocking. After this long of
 * back-to-back transfers the driver sleeps for a tick, so lower-priority tasks
 * on the same core, and its idle task that feeds the task watchdog, still run. */
#define PSRAM_BUSY_YIELD_US 20000
/* A gap this long between transfers counts as idle time. */
#define PSRAM_IDLE_GAP_US 1000

struct spi_psram {
    spi_psram_config_t config;
    spi_device_handle_t device;
    bool bus_ready;
    StaticSemaphore_t mutex_storage;
    SemaphoreHandle_t mutex;
    /* Transfers go through this internal DMA buffer, so callers may pass
     * buffers in PSRAM or anywhere else. */
    uint8_t *bounce;
    size_t burst_limit;
    uint32_t size;
    uint8_t id[PSRAM_ID_BYTES];
    int64_t busy_since_us;
    int64_t last_end_us;
};

static const char *TAG = "spi-psram";

size_t spi_psram_burst_limit(uint32_t clock_hz)
{
    if (clock_hz == 0U) {
        return SPI_PSRAM_BURST_MIN_BYTES;
    }
    /* Clocks available while chip-enable is low, less the fixed phases; the
     * quad data phase moves one byte every two clocks. */
    const uint64_t clocks = (uint64_t)clock_hz * SPI_PSRAM_CE_LOW_BUDGET_US / 1000000ULL;
    if (clocks <= SPI_PSRAM_BURST_OVERHEAD_CLOCKS) {
        return SPI_PSRAM_BURST_MIN_BYTES;
    }
    size_t bytes = (size_t)((clocks - SPI_PSRAM_BURST_OVERHEAD_CLOCKS) / 2U);
    bytes -= bytes % SPI_PSRAM_BURST_MIN_BYTES;
    if (bytes < SPI_PSRAM_BURST_MIN_BYTES) {
        bytes = SPI_PSRAM_BURST_MIN_BYTES;
    }
    return bytes > SPI_PSRAM_BURST_MAX_BYTES ? SPI_PSRAM_BURST_MAX_BYTES : bytes;
}

size_t spi_psram_next_burst(uint32_t address, size_t remaining, size_t limit)
{
    const size_t to_page_end = SPI_PSRAM_PAGE_BYTES - (address % SPI_PSRAM_PAGE_BYTES);
    size_t burst = remaining < limit ? remaining : limit;
    return burst < to_page_end ? burst : to_page_end;
}

static esp_err_t command_only(spi_psram_t *psram, uint8_t command)
{
    spi_transaction_ext_t transaction = {
        .base = {
            .flags = SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY,
            .cmd = command,
        },
        .address_bits = 0,
        .dummy_bits = 0,
    };
    return spi_device_polling_transmit(psram->device, &transaction.base);
}

static esp_err_t read_id(spi_psram_t *psram)
{
    memset(psram->bounce, 0, PSRAM_ID_BYTES);
    /* Single-line: command, a 24-bit dummy address, then eight ID bytes. */
    spi_transaction_ext_t transaction = {
        .base = {
            .flags = SPI_TRANS_VARIABLE_DUMMY,
            .cmd = PSRAM_CMD_READ_ID,
            .addr = 0,
            .length = 0,
            .rxlength = PSRAM_ID_BYTES * 8U,
            .rx_buffer = psram->bounce,
        },
        .dummy_bits = 0,
    };
    ESP_RETURN_ON_ERROR(spi_device_polling_transmit(psram->device, &transaction.base),
                        TAG, "read ID failed");
    memcpy(psram->id, psram->bounce, PSRAM_ID_BYTES);
    return ESP_OK;
}

static esp_err_t quad_read(spi_psram_t *psram, uint32_t address, size_t length)
{
    spi_transaction_ext_t transaction = {
        .base = {
            .flags = SPI_TRANS_MODE_QIO | SPI_TRANS_MULTILINE_ADDR | SPI_TRANS_VARIABLE_DUMMY,
            .cmd = PSRAM_CMD_FAST_QUAD_READ,
            .addr = address,
            .length = 0,
            .rxlength = length * 8U,
            .rx_buffer = psram->bounce,
        },
        .dummy_bits = PSRAM_QUAD_READ_DUMMY_CLOCKS,
    };
    return spi_device_polling_transmit(psram->device, &transaction.base);
}

static esp_err_t quad_write(spi_psram_t *psram, uint32_t address, size_t length)
{
    spi_transaction_ext_t transaction = {
        .base = {
            .flags = SPI_TRANS_MODE_QIO | SPI_TRANS_MULTILINE_ADDR | SPI_TRANS_VARIABLE_DUMMY,
            .cmd = PSRAM_CMD_QUAD_WRITE,
            .addr = address,
            .length = length * 8U,
            .tx_buffer = psram->bounce,
        },
        .dummy_bits = 0,
    };
    return spi_device_polling_transmit(psram->device, &transaction.base);
}

static bool id_plausible(const uint8_t id[PSRAM_ID_BYTES])
{
    return id[0] == PSRAM_ID_MANUFACTURER && id[1] == PSRAM_ID_KNOWN_GOOD_DIE;
}

static void release(spi_psram_t *psram)
{
    if (psram == NULL) {
        return;
    }
    if (psram->device != NULL) {
        (void)spi_bus_remove_device(psram->device);
    }
    if (psram->bus_ready) {
        (void)spi_bus_free(psram->config.host);
    }
    if (psram->bounce != NULL) {
        solar_os_memory_free(psram->bounce);
    }
    solar_os_memory_free(psram);
}

esp_err_t spi_psram_open(const spi_psram_config_t *config, spi_psram_t **out)
{
    if (config == NULL || out == NULL || config->clock_hz < SPI_PSRAM_CLOCK_MIN_HZ ||
        config->clock_hz > SPI_PSRAM_CLOCK_MAX_HZ ||
        !GPIO_IS_VALID_OUTPUT_GPIO(config->sclk_pin) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(config->cs_pin)) {
        return ESP_ERR_INVALID_ARG;
    }
    for (unsigned i = 0; i < 4U; i++) {
        if (!GPIO_IS_VALID_OUTPUT_GPIO(config->io_pins[i])) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    *out = NULL;

    spi_psram_t *psram = solar_os_memory_alloc(sizeof(*psram),
                                               SOLAR_OS_MEMORY_INTERNAL_PREFERRED,
                                               "psram");
    if (psram == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memset(psram, 0, sizeof(*psram));
    psram->config = *config;
    psram->burst_limit = spi_psram_burst_limit(config->clock_hz);
    psram->bounce = solar_os_memory_alloc(SPI_PSRAM_BURST_MAX_BYTES, SOLAR_OS_MEMORY_DMA,
                                          "psram.dma");
    if (psram->bounce == NULL) {
        release(psram);
        return ESP_ERR_NO_MEM;
    }
    psram->mutex = xSemaphoreCreateMutexStatic(&psram->mutex_storage);

    /* SIO0/SIO1 are the data in/out lines in single-line mode; SIO2/SIO3 are
     * the write-protect and hold lines the SPI peripheral drives in quad mode. */
    const spi_bus_config_t bus = {
        .mosi_io_num = config->io_pins[0],
        .miso_io_num = config->io_pins[1],
        .quadwp_io_num = config->io_pins[2],
        .quadhd_io_num = config->io_pins[3],
        .sclk_io_num = config->sclk_pin,
        .max_transfer_sz = SPI_PSRAM_BURST_MAX_BYTES,
    };
    esp_err_t err = spi_bus_initialize(config->host, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI host %d unavailable: %s", (int)config->host, esp_err_to_name(err));
        release(psram);
        return err;
    }
    psram->bus_ready = true;

    const spi_device_interface_config_t device = {
        .command_bits = 8,
        .address_bits = 24,
        .dummy_bits = 0,
        .mode = 0,
        .clock_speed_hz = (int)config->clock_hz,
        .spics_io_num = config->cs_pin,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 1,
    };
    err = spi_bus_add_device(config->host, &device, &psram->device);
    if (err != ESP_OK) {
        release(psram);
        return err;
    }

    /* The chip powers up in single-line SPI mode. Reset it in case a previous
     * run left it in an unknown state, then check that something answers. */
    err = command_only(psram, PSRAM_CMD_RESET_ENABLE);
    if (err == ESP_OK) {
        err = command_only(psram, PSRAM_CMD_RESET);
    }
    if (err == ESP_OK) {
        esp_rom_delay_us(PSRAM_RESET_DELAY_US);
        err = read_id(psram);
    }
    if (err != ESP_OK) {
        release(psram);
        return err;
    }
    if (!id_plausible(psram->id)) {
        ESP_LOGW(TAG, "no PSRAM: ID %02x %02x %02x %02x", psram->id[0], psram->id[1],
                 psram->id[2], psram->id[3]);
        release(psram);
        return ESP_ERR_NOT_FOUND;
    }
    psram->size = SPI_PSRAM_SIZE_BYTES;
    ESP_LOGI(TAG, "PSRAM found: ID %02x %02x %02x %02x %02x %02x %02x %02x, %u KiB, "
             "%lu MHz, bursts of %u bytes",
             psram->id[0], psram->id[1], psram->id[2], psram->id[3], psram->id[4],
             psram->id[5], psram->id[6], psram->id[7], (unsigned)(psram->size / 1024U),
             (unsigned long)(config->clock_hz / 1000000UL), (unsigned)psram->burst_limit);
    *out = psram;
    return ESP_OK;
}

void spi_psram_close(spi_psram_t *psram)
{
    release(psram);
}

uint32_t spi_psram_size(const spi_psram_t *psram)
{
    return psram != NULL ? psram->size : 0U;
}

uint32_t spi_psram_clock_hz(const spi_psram_t *psram)
{
    return psram != NULL ? psram->config.clock_hz : 0U;
}

const uint8_t *spi_psram_id(const spi_psram_t *psram)
{
    return psram != NULL ? psram->id : NULL;
}

static esp_err_t transfer(spi_psram_t *psram,
                          uint32_t address,
                          uint8_t *read_to,
                          const uint8_t *write_from,
                          size_t length)
{
    if (psram == NULL || (read_to == NULL) == (write_from == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (address > psram->size || length > psram->size - address) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (xSemaphoreTake(psram->mutex, pdMS_TO_TICKS(PSRAM_MUTEX_WAIT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (esp_timer_get_time() - psram->last_end_us > PSRAM_IDLE_GAP_US) {
        psram->busy_since_us = esp_timer_get_time();
    }
    /* Holding the bus for the whole transfer saves a lock round trip per burst. */
    esp_err_t err = spi_device_acquire_bus(psram->device, portMAX_DELAY);
    const bool acquired = err == ESP_OK;
    while (length > 0U && err == ESP_OK) {
        if (esp_timer_get_time() - psram->busy_since_us > PSRAM_BUSY_YIELD_US) {
            vTaskDelay(1);
            psram->busy_since_us = esp_timer_get_time();
        }
        const size_t burst = spi_psram_next_burst(address, length, psram->burst_limit);
        if (write_from != NULL) {
            memcpy(psram->bounce, write_from, burst);
            err = quad_write(psram, address, burst);
            write_from += burst;
        } else {
            err = quad_read(psram, address, burst);
            if (err == ESP_OK) {
                memcpy(read_to, psram->bounce, burst);
            }
            read_to += burst;
        }
        address += (uint32_t)burst;
        length -= burst;
    }
    if (acquired) {
        spi_device_release_bus(psram->device);
    }
    psram->last_end_us = esp_timer_get_time();
    xSemaphoreGive(psram->mutex);
    return err;
}

esp_err_t spi_psram_read(spi_psram_t *psram, uint32_t address, void *data, size_t length)
{
    return length == 0U ? ESP_OK : transfer(psram, address, data, NULL, length);
}

esp_err_t spi_psram_write(spi_psram_t *psram, uint32_t address, const void *data, size_t length)
{
    return length == 0U ? ESP_OK : transfer(psram, address, NULL, data, length);
}
