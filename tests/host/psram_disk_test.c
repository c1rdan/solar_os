#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "../../src/services/solar_os_psram_disk.c"
#include "../../src/services/solar_os_psram_disk_driver.c"

/* ---------------------------------------------------------------------------
 * A fake PSRAM chip with injectable faults: an address bit that is ignored (two
 * addresses share the same cells) or a data bit that always reads back as 1.
 * The disk-image layer is recorded, not run: its own test covers FatFS.
 * ------------------------------------------------------------------------ */

#define CHIP_BYTES (8U * 1024U * 1024U)

struct spi_psram {
    spi_psram_config_t config;
};

static uint8_t *chip;
static bool chip_present = true;
static uint32_t alias_bit;
static uint8_t stuck_high;
static unsigned opens, closes, open_handles;
static spi_psram_config_t last_config;
static struct spi_psram handle;

static int64_t fake_time_us;
int64_t esp_timer_get_time(void) { return fake_time_us += 7; }

static unsigned allocations;
void *solar_os_memory_alloc(size_t size, solar_os_memory_class_t memory_class, const char *tag)
{
    (void)memory_class;
    (void)tag;
    allocations++;
    return malloc(size);
}
void solar_os_memory_free(void *pointer)
{
    if (pointer != NULL) allocations--;
    free(pointer);
}

esp_err_t spi_psram_open(const spi_psram_config_t *config, spi_psram_t **out)
{
    opens++;
    last_config = *config;
    if (!chip_present) return ESP_ERR_NOT_FOUND;
    assert(open_handles == 0U);
    open_handles++;
    handle.config = *config;
    *out = &handle;
    return ESP_OK;
}
void spi_psram_close(spi_psram_t *psram)
{
    assert(psram == &handle && open_handles == 1U);
    open_handles--;
    closes++;
}
uint32_t spi_psram_size(const spi_psram_t *psram) { return psram != NULL ? CHIP_BYTES : 0U; }

static uint32_t cell(uint32_t address) { return address & ~alias_bit; }
esp_err_t spi_psram_read(spi_psram_t *psram, uint32_t address, void *data, size_t length)
{
    assert(psram == &handle && address + length <= CHIP_BYTES);
    for (size_t i = 0; i < length; i++) ((uint8_t *)data)[i] = chip[cell(address + (uint32_t)i)] | stuck_high;
    return ESP_OK;
}
esp_err_t spi_psram_write(spi_psram_t *psram, uint32_t address, const void *data, size_t length)
{
    assert(psram == &handle && address + length <= CHIP_BYTES);
    for (size_t i = 0; i < length; i++) chip[cell(address + (uint32_t)i)] = ((const uint8_t *)data)[i];
    return ESP_OK;
}

/* The disk-image layer. */
static unsigned attaches, detaches;
static esp_err_t attach_result = ESP_OK, detach_result = ESP_OK;
static const solar_os_disk_image_device_t *attached_device;
static void *attached_context;
esp_err_t solar_os_disk_image_attach_device(const char *name, const char *type_name,
                                            const solar_os_disk_image_device_t *device,
                                            void *context, uint32_t size, const char *mount_point,
                                            void *work, size_t work_size)
{
    attaches++;
    assert(!strcmp(name, "psram0") && !strcmp(type_name, "psram"));
    assert(size == CHIP_BYTES && !strcmp(mount_point, "/psram"));
    assert(work != NULL && work_size >= SOLAR_OS_DISK_IMAGE_FORMAT_WORK_BYTES);
    assert(device != NULL && device->read != NULL && device->write != NULL);
    attached_device = device;
    attached_context = context;
    return attach_result;
}
esp_err_t solar_os_disk_image_detach_device(const char *mount_point)
{
    assert(!strcmp(mount_point, "/psram"));
    detaches++;
    return detach_result;
}

/* ---------------------------------------------------------------------------
 * Tests
 * ------------------------------------------------------------------------ */

static solar_os_expansion_binding_t bindings[8];
static size_t binding_count;

static void bind(solar_os_expansion_binding_kind_t kind, const char *role, int value)
{
    solar_os_expansion_binding_t *binding = &bindings[binding_count++];
    memset(binding, 0, sizeof(*binding));
    binding->kind = kind;
    strncpy(binding->role, role, sizeof(binding->role) - 1U);
    binding->value = value;
}

/* The PicoCalc wiring, optionally without one role and with host and speed. */
static void picocalc_bindings(const char *omit, int host, int speed)
{
    static const struct { const char *role; int gpio; } pins[] = {
        {"sclk", 5}, {"cs", 4}, {"io0", 13}, {"io1", 14}, {"io2", 15}, {"io3", 16},
    };
    binding_count = 0;
    for (unsigned i = 0; i < 6U; i++) {
        if (omit == NULL || strcmp(omit, pins[i].role) != 0) {
            bind(SOLAR_OS_EXPANSION_BINDING_GPIO, pins[i].role, pins[i].gpio);
        }
    }
    if (host != 0) bind(SOLAR_OS_EXPANSION_BINDING_PARAMETER, "host", host);
    if (speed != 0) bind(SOLAR_OS_EXPANSION_BINDING_PARAMETER, "speed", speed);
}

static void reset(void)
{
    memset(chip, 0, CHIP_BYTES);
    chip_present = true;
    alias_bit = 0U;
    stuck_high = 0U;
    attach_result = ESP_OK;
    detach_result = ESP_OK;
    opens = closes = attaches = detaches = 0U;
}

static esp_err_t attach(void) { return solar_os_psram_disk_attach("psram0", bindings, binding_count); }

static void test_healthy_chip_mounts(void)
{
    reset();
    picocalc_bindings(NULL, 3, 32);
    assert(attach() == ESP_OK);
    assert(opens == 1U && attaches == 1U && open_handles == 1U);
    assert(last_config.host == SPI3_HOST && last_config.clock_hz == 32000000U);
    assert(last_config.sclk_pin == 5 && last_config.cs_pin == 4);
    assert(last_config.io_pins[0] == 13 && last_config.io_pins[1] == 14);
    assert(last_config.io_pins[2] == 15 && last_config.io_pins[3] == 16);
    /* The work buffer is only borrowed for the self-test and formatting. */
    assert(allocations == 0U);
    /* The volume reads and writes the chip through the driver. */
    assert(attached_context == &handle);
    uint8_t sector[512];
    memset(sector, 0x3c, sizeof(sector));
    assert(attached_device->write(attached_context, 4096U, sector, sizeof(sector)) == ESP_OK);
    memset(sector, 0, sizeof(sector));
    assert(attached_device->read(attached_context, 4096U, sector, sizeof(sector)) == ESP_OK);
    assert(sector[0] == 0x3c && sector[511] == 0x3c);

    /* One chip owns the host and the mount point. */
    assert(attach() == ESP_ERR_INVALID_STATE && opens == 1U);
    assert(solar_os_psram_disk_detach("psram1") == ESP_ERR_NOT_FOUND);
    assert(solar_os_psram_disk_detach("psram0") == ESP_OK);
    assert(detaches == 1U && closes == 1U && open_handles == 0U);
    assert(solar_os_psram_disk_detach("psram0") == ESP_ERR_NOT_FOUND);
}

static void test_detach_after_shutdown_unmount(void)
{
    /* The shutdown path may already have unmounted the volume. */
    reset();
    picocalc_bindings(NULL, 0, 0);
    assert(attach() == ESP_OK);
    /* host and speed default to SPI3 at the driver's default clock. */
    assert(last_config.host == SPI3_HOST && last_config.clock_hz == SPI_PSRAM_CLOCK_DEFAULT_HZ);
    detach_result = ESP_ERR_NOT_FOUND;
    assert(solar_os_psram_disk_detach("psram0") == ESP_OK && closes == 1U && open_handles == 0U);
    /* Any other unmount failure keeps the chip open for a retry. */
    detach_result = ESP_OK;
    assert(attach() == ESP_OK);
    detach_result = ESP_FAIL;
    assert(solar_os_psram_disk_detach("psram0") == ESP_FAIL && open_handles == 1U);
    detach_result = ESP_OK;
    assert(solar_os_psram_disk_detach("psram0") == ESP_OK && open_handles == 0U);
}

static void test_bindings(void)
{
    reset();
    picocalc_bindings("io3", 3, 32);
    assert(attach() == ESP_ERR_INVALID_ARG && opens == 0U);
    picocalc_bindings("cs", 3, 32);
    assert(attach() == ESP_ERR_INVALID_ARG && opens == 0U);
    picocalc_bindings(NULL, 4, 32);
    assert(attach() == ESP_ERR_INVALID_ARG && opens == 0U);
    picocalc_bindings(NULL, 2, 26);
    assert(attach() == ESP_OK);
    assert(last_config.host == SPI2_HOST && last_config.clock_hz == 26000000U);
    assert(solar_os_psram_disk_detach("psram0") == ESP_OK);
    assert(solar_os_psram_disk_attach(NULL, bindings, binding_count) == ESP_ERR_INVALID_ARG);
}

/* A chip that fails a check is closed and never formatted. */
static void expect_rejected(esp_err_t expected)
{
    picocalc_bindings(NULL, 3, 32);
    assert(attach() == expected);
    assert(attaches == 0U && open_handles == 0U && allocations == 0U);
    assert(!disk.attached);
}

static void test_faulty_chips_are_rejected(void)
{
    /* Every address line from A0 to A22, including those only the pattern spans
     * reach. */
    for (uint32_t bit = 1U; bit < CHIP_BYTES; bit <<= 1) {
        reset();
        alias_bit = bit;
        expect_rejected(ESP_ERR_INVALID_RESPONSE);
    }
    for (unsigned bit = 0; bit < 8U; bit++) {
        reset();
        stuck_high = (uint8_t)(1U << bit);
        expect_rejected(ESP_ERR_INVALID_RESPONSE);
    }
    reset();
    chip_present = false;
    expect_rejected(ESP_ERR_NOT_FOUND);
    assert(opens == 1U && closes == 0U);
    /* A chip that passes but fails to mount is released too, and a later attach
     * works. */
    reset();
    attach_result = ESP_FAIL;
    picocalc_bindings(NULL, 3, 32);
    assert(attach() == ESP_FAIL);
    assert(attaches == 1U && closes == 1U && open_handles == 0U && allocations == 0U);
    assert(!disk.attached);
    reset();
    picocalc_bindings(NULL, 3, 32);
    assert(attach() == ESP_OK);
    assert(solar_os_psram_disk_detach("psram0") == ESP_OK);
}

static void test_driver_description(void)
{
    const solar_os_expansion_driver_t *driver = &solar_os_psram_disk_expansion_driver;
    assert(!strcmp(driver->name, "psram-disk"));
    assert(driver->category == SOLAR_OS_EXPANSION_CATEGORY_STORAGE);
    assert(driver->attach == solar_os_psram_disk_attach && driver->detach == solar_os_psram_disk_detach);
    unsigned required = 0U;
    for (size_t i = 0; i < driver->binding_spec_count; i++) {
        const solar_os_expansion_binding_spec_t *spec = &driver->binding_specs[i];
        required += spec->required;
        if (!strcmp(spec->key, "speed")) {
            assert(spec->has_value_range && spec->min_value == 10 && spec->max_value == 40);
        }
        if (!strcmp(spec->key, "host")) {
            assert(spec->has_value_range && spec->min_value == 2 && spec->max_value == 3);
        }
    }
    assert(required == 6U);
}

int main(void)
{
    chip = malloc(CHIP_BYTES);
    assert(chip != NULL);
    test_healthy_chip_mounts();
    test_detach_after_shutdown_unmount();
    test_bindings();
    test_faulty_chips_are_rejected();
    test_driver_description();
    free(chip);
    puts("psram_disk_test: ok");
    return 0;
}
