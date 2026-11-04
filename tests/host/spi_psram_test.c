#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/spi_master.h"
#include "solar_os_memory.h"
#include "spi_psram.h"

/* ---------------------------------------------------------------------------
 * A model of the PSRAM chip that enforces what the real one does: command
 * encodings, quad-line transactions, the wait cycles of Fast Quad Read, the
 * 1 KiB page boundary and the limit on how long chip-enable may stay low.
 * ------------------------------------------------------------------------ */

#define CHIP_BYTES SPI_PSRAM_SIZE_BYTES
/* Datasheet tCEM at 85 C, in nanoseconds. The driver budgets less than this. */
#define CE_LOW_LIMIT_NS 8000ULL

static uint8_t chip_memory[CHIP_BYTES];
static uint8_t chip_id[8] = {0x0d, 0x5d, 0x52, 0x34, 0x56, 0x78, 0x9a, 0xbc};
static bool chip_present;
static bool reset_enabled;
static unsigned reset_count, id_reads, quad_reads, quad_writes;
static size_t largest_burst;
static unsigned sequence;
static unsigned reset_enable_seq, reset_seq, id_seq;

static struct { bool initialized; spi_host_device_t host; spi_bus_config_t bus; } bus_state;
static struct {
    bool added;
    spi_device_interface_config_t config;
} device_state;
static unsigned device_removals, bus_frees;
static bool fail_bus_init;

struct spi_device { int unused; };
static struct spi_device the_device;

const char *esp_err_to_name(esp_err_t err) { (void)err; return "error"; }
void esp_rom_delay_us(uint32_t us) { (void)us; }

void *solar_os_memory_alloc(size_t size, solar_os_memory_class_t memory_class, const char *tag)
{
    (void)memory_class;
    (void)tag;
    return malloc(size);
}
void solar_os_memory_free(void *pointer) { free(pointer); }

esp_err_t spi_bus_initialize(spi_host_device_t host, const spi_bus_config_t *config, int dma)
{
    (void)dma;
    if (fail_bus_init || bus_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    bus_state.initialized = true;
    bus_state.host = host;
    bus_state.bus = *config;
    return ESP_OK;
}

esp_err_t spi_bus_free(spi_host_device_t host)
{
    assert(bus_state.initialized && bus_state.host == host);
    bus_state.initialized = false;
    bus_frees++;
    return ESP_OK;
}

esp_err_t spi_bus_add_device(spi_host_device_t host,
                             const spi_device_interface_config_t *config,
                             spi_device_handle_t *handle)
{
    assert(bus_state.initialized && bus_state.host == host && !device_state.added);
    device_state.added = true;
    device_state.config = *config;
    *handle = &the_device;
    return ESP_OK;
}

esp_err_t spi_bus_remove_device(spi_device_handle_t handle)
{
    assert(handle == &the_device && device_state.added);
    device_state.added = false;
    device_removals++;
    return ESP_OK;
}

static void check_ce_time(size_t clocks)
{
    const uint64_t ns = (uint64_t)clocks * 1000000000ULL /
        (uint64_t)device_state.config.clock_speed_hz;
    assert(ns <= CE_LOW_LIMIT_NS);
}

esp_err_t spi_device_polling_transmit(spi_device_handle_t handle, spi_transaction_t *transaction)
{
    assert(handle == &the_device && device_state.added);
    /* The driver must configure a half-duplex 8-bit command, 24-bit address. */
    assert(device_state.config.flags & SPI_DEVICE_HALFDUPLEX);
    assert(device_state.config.command_bits == 8 && device_state.config.address_bits == 24);
    assert(device_state.config.mode == 0);

    const spi_transaction_ext_t *ext = (const spi_transaction_ext_t *)transaction;
    const unsigned cmd = transaction->cmd;
    if (!chip_present) {
        if (transaction->rx_buffer != NULL) {
            memset(transaction->rx_buffer, 0xff, transaction->rxlength / 8U);
        }
        return ESP_OK;
    }

    switch (cmd) {
    case 0x66:
        assert(ext->address_bits == 0 && (transaction->flags & SPI_TRANS_VARIABLE_ADDR));
        reset_enabled = true;
        reset_enable_seq = ++sequence;
        return ESP_OK;
    case 0x99:
        assert(ext->address_bits == 0 && (transaction->flags & SPI_TRANS_VARIABLE_ADDR));
        assert(reset_enabled);
        reset_enabled = false;
        reset_count++;
        reset_seq = ++sequence;
        return ESP_OK;
    case 0x9f:
        /* Single-line ID read: no quad mode, eight bytes. */
        assert(!(transaction->flags & SPI_TRANS_MODE_QIO));
        assert(transaction->rxlength == 64 && transaction->length == 0);
        assert(ext->dummy_bits == 0);
        id_reads++;
        id_seq = ++sequence;
        memcpy(transaction->rx_buffer, chip_id, 8);
        return ESP_OK;
    case 0xeb:
    case 0x38: {
        const bool is_read = cmd == 0xeb;
        /* Command on one line, then the address and data on all four. */
        assert(transaction->flags & SPI_TRANS_MODE_QIO);
        assert(transaction->flags & SPI_TRANS_MULTILINE_ADDR);
        assert(transaction->flags & SPI_TRANS_VARIABLE_DUMMY);
        assert(ext->dummy_bits == (is_read ? 6 : 0));
        const size_t bits = is_read ? transaction->rxlength : transaction->length;
        assert(bits > 0 && bits % 8U == 0);
        if (is_read) {
            assert(transaction->length == 0 && transaction->rx_buffer != NULL);
        } else {
            assert(transaction->rxlength == 0 && transaction->tx_buffer != NULL);
        }
        const size_t bytes = bits / 8U;
        const uint32_t address = (uint32_t)transaction->addr;
        assert(address < CHIP_BYTES && bytes <= CHIP_BYTES - address);
        /* Bursts must stay within a page and within the CE-low time. */
        assert(address / SPI_PSRAM_PAGE_BYTES == (address + bytes - 1U) / SPI_PSRAM_PAGE_BYTES);
        /* The DMA engine wants whole words. */
        assert(bytes % 4U == 0U);
        /* Eight command clocks, six address clocks, the wait cycles, then two
         * clocks per data byte. */
        check_ce_time(8U + 6U + ext->dummy_bits + bytes * 2U);
        if (bytes > largest_burst) {
            largest_burst = bytes;
        }
        if (is_read) {
            quad_reads++;
            memcpy(transaction->rx_buffer, &chip_memory[address], bytes);
        } else {
            quad_writes++;
            memcpy(&chip_memory[address], transaction->tx_buffer, bytes);
        }
        return ESP_OK;
    }
    default:
        assert(!"unexpected command");
        return ESP_FAIL;
    }
}

/* ---------------------------------------------------------------------------
 * Tests
 * ------------------------------------------------------------------------ */

static spi_psram_config_t make_config(uint32_t clock_hz)
{
    return (spi_psram_config_t) {
        .host = SPI3_HOST,
        .sclk_pin = 5,
        .cs_pin = 4,
        .io_pins = {13, 14, 15, 16},
        .clock_hz = clock_hz,
    };
}

static void reset_model(void)
{
    memset(chip_memory, 0, sizeof(chip_memory));
    chip_present = true;
    reset_enabled = false;
    reset_count = id_reads = quad_reads = quad_writes = 0U;
    largest_burst = 0U;
    sequence = reset_enable_seq = reset_seq = id_seq = 0U;
    memset(&bus_state, 0, sizeof(bus_state));
    memset(&device_state, 0, sizeof(device_state));
    device_removals = bus_frees = 0U;
    fail_bus_init = false;
}

static void test_burst_limit(void)
{
    size_t previous = 0U;
    static const uint32_t clocks[] = {1000000U, 5000000U, 10000000U, 20000000U,
                                      26000000U, 40000000U};
    for (unsigned i = 0; i < sizeof(clocks) / sizeof(clocks[0]); i++) {
        const size_t limit = spi_psram_burst_limit(clocks[i]);
        assert(limit >= SPI_PSRAM_BURST_MIN_BYTES && limit <= SPI_PSRAM_BURST_MAX_BYTES);
        assert(limit % SPI_PSRAM_BURST_MIN_BYTES == 0U);
        assert(limit >= previous);
        previous = limit;
        /* A slower clock must not buy a longer burst than the datasheet allows
         * (except where the minimum burst already exceeds it). */
        const uint64_t ns = (uint64_t)(8U + 6U + 6U + limit * 2U) * 1000000000ULL / clocks[i];
        if (limit > SPI_PSRAM_BURST_MIN_BYTES) {
            assert(ns <= CE_LOW_LIMIT_NS);
        }
    }
    assert(spi_psram_burst_limit(0U) == SPI_PSRAM_BURST_MIN_BYTES);
    /* The practical clocks keep useful bursts. */
    assert(spi_psram_burst_limit(20000000U) >= 32U);
    assert(spi_psram_burst_limit(40000000U) >= 64U);
}

static void test_next_burst(void)
{
    assert(spi_psram_next_burst(0U, 1000U, 128U) == 128U);
    assert(spi_psram_next_burst(0U, 50U, 128U) == 50U);
    /* Never across a page. */
    assert(spi_psram_next_burst(1024U - 16U, 1000U, 128U) == 16U);
    assert(spi_psram_next_burst(1024U - 200U, 1000U, 128U) == 128U);
    assert(spi_psram_next_burst(1024U - 100U, 1000U, 128U) == 100U);
    assert(spi_psram_next_burst(1023U, 5U, 128U) == 1U);
    assert(spi_psram_next_burst(1024U, 5000U, 96U) == 96U);
    for (uint32_t address = 0U; address < 4096U; address += 7U) {
        const size_t burst = spi_psram_next_burst(address, 4096U - address, 112U);
        assert(burst >= 1U && burst <= 112U);
        assert(address / SPI_PSRAM_PAGE_BYTES == (address + burst - 1U) / SPI_PSRAM_PAGE_BYTES);
    }
}

static void test_open_and_close(void)
{
    reset_model();
    spi_psram_config_t config = make_config(20000000U);
    spi_psram_t *psram = NULL;
    assert(spi_psram_open(&config, &psram) == ESP_OK && psram != NULL);

    /* Pins map to the bus the way the chip is wired: SIO0/SIO1 are MOSI/MISO,
     * SIO2/SIO3 the write-protect and hold lines. */
    assert(bus_state.host == SPI3_HOST);
    assert(bus_state.bus.mosi_io_num == 13 && bus_state.bus.miso_io_num == 14);
    assert(bus_state.bus.quadwp_io_num == 15 && bus_state.bus.quadhd_io_num == 16);
    assert(bus_state.bus.sclk_io_num == 5);
    assert(device_state.config.spics_io_num == 4);
    assert(device_state.config.clock_speed_hz == 20000000);

    /* Reset enable, reset, then the ID read, in that order. */
    assert(reset_count == 1U && id_reads == 1U);
    assert(reset_enable_seq < reset_seq && reset_seq < id_seq);

    assert(spi_psram_size(psram) == SPI_PSRAM_SIZE_BYTES);
    assert(spi_psram_clock_hz(psram) == 20000000U);
    assert(spi_psram_id(psram)[0] == 0x0d && spi_psram_id(psram)[1] == 0x5d);

    spi_psram_close(psram);
    assert(!device_state.added && !bus_state.initialized);
    assert(device_removals == 1U && bus_frees == 1U);
    spi_psram_close(NULL);
}

static void test_open_failures(void)
{
    spi_psram_t *psram = NULL;

    /* No chip: every line floats high, so the ID is all ones. */
    reset_model();
    chip_present = false;
    spi_psram_config_t config = make_config(20000000U);
    assert(spi_psram_open(&config, &psram) == ESP_ERR_NOT_FOUND && psram == NULL);
    assert(!device_state.added && !bus_state.initialized && bus_frees == 1U);

    /* Something answers but is not this chip. */
    reset_model();
    chip_id[0] = 0x1f;
    assert(spi_psram_open(&config, &psram) == ESP_ERR_NOT_FOUND && psram == NULL);
    assert(!bus_state.initialized);
    chip_id[0] = 0x0d;

    /* The SPI host is unavailable. */
    reset_model();
    fail_bus_init = true;
    assert(spi_psram_open(&config, &psram) == ESP_ERR_INVALID_STATE && psram == NULL);
    assert(!device_state.added && bus_frees == 0U);

    /* Bad configuration is rejected before touching the hardware. */
    reset_model();
    assert(spi_psram_open(NULL, &psram) == ESP_ERR_INVALID_ARG);
    assert(spi_psram_open(&config, NULL) == ESP_ERR_INVALID_ARG);
    spi_psram_config_t bad = config;
    bad.clock_hz = 0U;
    assert(spi_psram_open(&bad, &psram) == ESP_ERR_INVALID_ARG);
    bad.clock_hz = SPI_PSRAM_CLOCK_MIN_HZ - 1U;
    assert(spi_psram_open(&bad, &psram) == ESP_ERR_INVALID_ARG);
    bad = config;
    bad.clock_hz = SPI_PSRAM_CLOCK_MAX_HZ + 1U;
    assert(spi_psram_open(&bad, &psram) == ESP_ERR_INVALID_ARG);
    bad = config;
    bad.io_pins[3] = -1;
    assert(spi_psram_open(&bad, &psram) == ESP_ERR_INVALID_ARG);
    bad = config;
    bad.cs_pin = 99;
    assert(spi_psram_open(&bad, &psram) == ESP_ERR_INVALID_ARG);
    assert(!bus_state.initialized);
}

static void fill_pattern(uint8_t *buffer, size_t length, unsigned seed)
{
    unsigned value = seed * 2654435761U + 12345U;
    for (size_t i = 0; i < length; i++) {
        value = value * 1103515245U + 12345U;
        buffer[i] = (uint8_t)(value >> 16);
    }
}

static void check_round_trip(spi_psram_t *psram, uint32_t address, size_t length, unsigned seed)
{
    uint8_t *written = calloc(1, length);
    uint8_t *read_back = calloc(1, length);
    assert(written != NULL && read_back != NULL);
    fill_pattern(written, length, seed);
    /* Poison the destination so a skipped read cannot pass by accident. */
    memset(read_back, 0xa5, length);
    assert(spi_psram_write(psram, address, written, length) == ESP_OK);
    /* The chip's memory must hold exactly the bytes at exactly the addresses. */
    assert(memcmp(&chip_memory[address], written, length) == 0);
    assert(spi_psram_read(psram, address, read_back, length) == ESP_OK);
    assert(memcmp(read_back, written, length) == 0);
    free(written);
    free(read_back);
}

static void test_transfers_at_clock(uint32_t clock_hz)
{
    reset_model();
    spi_psram_config_t config = make_config(clock_hz);
    spi_psram_t *psram = NULL;
    assert(spi_psram_open(&config, &psram) == ESP_OK);

    /* Sector-sized, page-crossing, odd-length and tail-of-memory transfers.
     * Lengths that are not word multiples only occur at a page edge, which is
     * the case that must still be whole words on the wire. */
    check_round_trip(psram, 0U, 512U, 1U);
    check_round_trip(psram, 512U, 512U, 2U);
    check_round_trip(psram, 1024U - 64U, 128U, 3U);
    check_round_trip(psram, 4096U + 16U, 4096U, 4U);
    check_round_trip(psram, 100U * 1024U + 4U, 10000U * 4U / 4U, 5U);
    check_round_trip(psram, 1024U * 1024U, 65536U, 6U);
    check_round_trip(psram, CHIP_BYTES - 512U, 512U, 7U);
    check_round_trip(psram, CHIP_BYTES / 2U + 12U, 20U, 8U);

    /* Bursts honour the budget for this clock. */
    assert(largest_burst <= spi_psram_burst_limit(clock_hz));
    assert(largest_burst >= SPI_PSRAM_BURST_MIN_BYTES);
    assert(quad_reads > 0U && quad_writes > 0U);
    spi_psram_close(psram);
}

static void test_bounds_and_arguments(void)
{
    reset_model();
    spi_psram_config_t config = make_config(20000000U);
    spi_psram_t *psram = NULL;
    assert(spi_psram_open(&config, &psram) == ESP_OK);
    uint8_t buffer[64] = {0};

    assert(spi_psram_read(psram, CHIP_BYTES - 32U, buffer, 64U) == ESP_ERR_INVALID_SIZE);
    assert(spi_psram_write(psram, CHIP_BYTES - 32U, buffer, 64U) == ESP_ERR_INVALID_SIZE);
    assert(spi_psram_read(psram, CHIP_BYTES + 1U, buffer, 1U) == ESP_ERR_INVALID_SIZE);
    assert(spi_psram_read(psram, UINT32_MAX, buffer, 64U) == ESP_ERR_INVALID_SIZE);
    /* Exactly the last bytes are fine. */
    assert(spi_psram_write(psram, CHIP_BYTES - 64U, buffer, 64U) == ESP_OK);
    assert(spi_psram_read(psram, CHIP_BYTES - 64U, buffer, 64U) == ESP_OK);
    assert(spi_psram_read(psram, 0U, NULL, 16U) == ESP_ERR_INVALID_ARG);
    assert(spi_psram_write(psram, 0U, NULL, 16U) == ESP_ERR_INVALID_ARG);
    assert(spi_psram_read(NULL, 0U, buffer, 16U) == ESP_ERR_INVALID_ARG);
    /* Zero length does nothing, even with a bad pointer. */
    const unsigned reads = quad_reads, writes = quad_writes;
    assert(spi_psram_read(psram, 0U, NULL, 0U) == ESP_OK);
    assert(spi_psram_write(psram, 0U, NULL, 0U) == ESP_OK);
    assert(quad_reads == reads && quad_writes == writes);
    spi_psram_close(psram);
}

int main(void)
{
    test_burst_limit();
    test_next_burst();
    test_open_and_close();
    test_open_failures();
    /* From the slowest permitted clock up to the matrix limit. */
    test_transfers_at_clock(SPI_PSRAM_CLOCK_MIN_HZ);
    test_transfers_at_clock(20000000U);
    test_transfers_at_clock(26000000U);
    test_transfers_at_clock(40000000U);
    test_bounds_and_arguments();
    puts("spi_psram_test: ok");
    return 0;
}
