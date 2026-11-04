#include "solar_os_psram_disk.h"

#include "spi_psram.h"

static const solar_os_expansion_binding_spec_t binding_specs[] = {
    {.key = "sclk", .value_hint = "gpio", .kind = SOLAR_OS_EXPANSION_BINDING_GPIO, .role = "sclk", .required = true},
    {.key = "cs", .value_hint = "gpio", .kind = SOLAR_OS_EXPANSION_BINDING_GPIO, .role = "cs", .required = true},
    {.key = "io0", .value_hint = "gpio", .kind = SOLAR_OS_EXPANSION_BINDING_GPIO, .role = "io0", .required = true},
    {.key = "io1", .value_hint = "gpio", .kind = SOLAR_OS_EXPANSION_BINDING_GPIO, .role = "io1", .required = true},
    {.key = "io2", .value_hint = "gpio", .kind = SOLAR_OS_EXPANSION_BINDING_GPIO, .role = "io2", .required = true},
    {.key = "io3", .value_hint = "gpio", .kind = SOLAR_OS_EXPANSION_BINDING_GPIO, .role = "io3", .required = true},
    {.key = "host", .value_hint = "2|3", .kind = SOLAR_OS_EXPANSION_BINDING_PARAMETER, .role = "host",
     .has_value_range = true, .min_value = 2, .max_value = 3},
    {.key = "speed", .value_hint = "MHz", .kind = SOLAR_OS_EXPANSION_BINDING_PARAMETER, .role = "speed",
     .has_value_range = true, .min_value = (int)(SPI_PSRAM_CLOCK_MIN_HZ / 1000000U),
     .max_value = (int)(SPI_PSRAM_CLOCK_MAX_HZ / 1000000U)},
};

const solar_os_expansion_driver_t solar_os_psram_disk_expansion_driver = {
    .name = "psram-disk",
    .category = SOLAR_OS_EXPANSION_CATEGORY_STORAGE,
    .summary = "SPI PSRAM chip as a RAM disk at /psram",
    .required_capabilities = SOLAR_OS_BOARD_CAP_EXPANSION_GPIO,
    .binding_specs = binding_specs,
    .binding_spec_count = sizeof(binding_specs) / sizeof(binding_specs[0]),
    .attach = solar_os_psram_disk_attach,
    .detach = solar_os_psram_disk_detach,
};
