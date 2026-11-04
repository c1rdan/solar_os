#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Driver for a serial PSRAM chip (APS6404L / ESP-PSRAM64H class) on its own SPI
 * host, using the quad lines. This is an ordinary SPI peripheral, not memory
 * mapped: the ESP32-S3's own PSRAM interface cannot reach a chip wired to
 * general GPIOs, so the chip is read and written in small bursts.
 *
 * Reads use Fast Quad Read (0xEB) and writes Quad Write (0x38): the command
 * goes out on one line, then the 24-bit address and the data on all four.
 *
 * Two limits shape the bursts. The chip refreshes itself, so chip-enable may
 * stay low for only a few microseconds (the datasheet's tCEM); the burst is
 * therefore sized from the SPI clock. And a burst must not cross a 1 KiB page
 * boundary. */
#define SPI_PSRAM_SIZE_BYTES (8U * 1024U * 1024U)
#define SPI_PSRAM_PAGE_BYTES 1024U
#define SPI_PSRAM_BURST_MIN_BYTES 16U
#define SPI_PSRAM_BURST_MAX_BYTES 128U
/* Longest time to hold chip-enable low, with margin under the datasheet limit
 * of 8 microseconds at 85 C. */
#define SPI_PSRAM_CE_LOW_BUDGET_US 6U
/* Clocks spent on the command, address and dummy phases of a burst. */
#define SPI_PSRAM_BURST_OVERHEAD_CLOCKS 20U
/* The GPIO matrix limits SPI master clocks to 40 MHz. Below 10 MHz even the
 * smallest burst keeps chip-enable low longer than the chip allows. */
#define SPI_PSRAM_CLOCK_MIN_HZ 10000000U
#define SPI_PSRAM_CLOCK_MAX_HZ 40000000U
#define SPI_PSRAM_CLOCK_DEFAULT_HZ 20000000U

typedef struct {
    spi_host_device_t host;
    int sclk_pin;
    int cs_pin;
    /* SIO0 (SI), SIO1 (SO), SIO2, SIO3. */
    int io_pins[4];
    uint32_t clock_hz;
} spi_psram_config_t;

typedef struct spi_psram spi_psram_t;

/* Largest burst that keeps chip-enable low within the budget at this clock,
 * a multiple of SPI_PSRAM_BURST_MIN_BYTES within [MIN, MAX]. */
size_t spi_psram_burst_limit(uint32_t clock_hz);
/* Size of the next burst starting at address: at most remaining and limit
 * bytes, and never across a page boundary. */
size_t spi_psram_next_burst(uint32_t address, size_t remaining, size_t limit);

/* Reset the chip, read and check its ID, and return a handle. ESP_ERR_NOT_FOUND
 * means no chip answered with a plausible ID. */
esp_err_t spi_psram_open(const spi_psram_config_t *config, spi_psram_t **out);
void spi_psram_close(spi_psram_t *psram);
uint32_t spi_psram_size(const spi_psram_t *psram);
uint32_t spi_psram_clock_hz(const spi_psram_t *psram);
/* ID bytes read at open: manufacturer, known-good-die, then the 6 EID bytes. */
const uint8_t *spi_psram_id(const spi_psram_t *psram);

esp_err_t spi_psram_read(spi_psram_t *psram, uint32_t address, void *data, size_t length);
esp_err_t spi_psram_write(spi_psram_t *psram, uint32_t address, const void *data, size_t length);

#ifdef __cplusplus
}
#endif
