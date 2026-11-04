#include "solar_os_picocalc_core.h"

static const int addresses[] = {SOLAR_OS_PICOCALC_CORE_ADDRESS};
static const solar_os_expansion_binding_spec_t binding_specs[] = {
    {.key = "i2c", .value_hint = "bus", .kind = SOLAR_OS_EXPANSION_BINDING_I2C_BUS, .required = true},
    {.key = "addr", .value_hint = "0x1f", .kind = SOLAR_OS_EXPANSION_BINDING_I2C_ADDRESS, .required = true, .allowed_values = addresses, .allowed_value_count = sizeof(addresses) / sizeof(addresses[0])},
    {.key = "gap", .value_hint = "ms", .kind = SOLAR_OS_EXPANSION_BINDING_PARAMETER, .role = "gap", .has_value_range = true, .min_value = 0, .max_value = SOLAR_OS_PICOCALC_CORE_MAX_GAP_MS},
};

const solar_os_expansion_driver_t solar_os_picocalc_core_expansion_driver = {
    .name = "picocalc-core",
    .category = SOLAR_OS_EXPANSION_CATEGORY_UTILITY,
    .summary = "Clockwork PicoCalc keyboard and power controller",
    .required_capabilities = SOLAR_OS_BOARD_CAP_I2C,
    .probe_supported = false,
    .binding_specs = binding_specs,
    .binding_spec_count = sizeof(binding_specs) / sizeof(binding_specs[0]),
    .attach = solar_os_picocalc_core_attach,
    .detach = solar_os_picocalc_core_detach,
};
