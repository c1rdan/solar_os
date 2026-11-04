#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

typedef int spi_host_device_t;
#define SPI2_HOST 1
#define SPI3_HOST 2
#define SPI_DMA_CH_AUTO 3

typedef struct spi_device *spi_device_handle_t;

typedef struct {
    int mosi_io_num;
    int miso_io_num;
    int sclk_io_num;
    int quadwp_io_num;
    int quadhd_io_num;
    int max_transfer_sz;
} spi_bus_config_t;

#define SPI_DEVICE_HALFDUPLEX (1U << 4)

typedef struct {
    uint8_t command_bits;
    uint8_t address_bits;
    uint8_t dummy_bits;
    uint8_t mode;
    int clock_speed_hz;
    int spics_io_num;
    uint32_t flags;
    int queue_size;
} spi_device_interface_config_t;

#define SPI_TRANS_MODE_QIO (1U << 0)
#define SPI_TRANS_MULTILINE_ADDR (1U << 1)
#define SPI_TRANS_VARIABLE_ADDR (1U << 2)
#define SPI_TRANS_VARIABLE_DUMMY (1U << 3)

typedef struct spi_transaction_t {
    uint32_t flags;
    uint16_t cmd;
    uint64_t addr;
    size_t length;
    size_t rxlength;
    void *user;
    union {
        const void *tx_buffer;
        uint8_t tx_data[4];
    };
    union {
        void *rx_buffer;
        uint8_t rx_data[4];
    };
} spi_transaction_t;

typedef struct {
    struct spi_transaction_t base;
    uint8_t command_bits;
    uint8_t address_bits;
    uint8_t dummy_bits;
} spi_transaction_ext_t;

esp_err_t spi_bus_initialize(spi_host_device_t host, const spi_bus_config_t *config, int dma);
esp_err_t spi_bus_free(spi_host_device_t host);
esp_err_t spi_bus_add_device(spi_host_device_t host,
                             const spi_device_interface_config_t *config,
                             spi_device_handle_t *handle);
esp_err_t spi_bus_remove_device(spi_device_handle_t handle);
esp_err_t spi_device_polling_transmit(spi_device_handle_t handle, spi_transaction_t *transaction);
esp_err_t spi_device_acquire_bus(spi_device_handle_t handle, TickType_t wait);
void spi_device_release_bus(spi_device_handle_t handle);
