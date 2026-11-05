#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "solar_os_expansion.h"

/* Clockwork PicoCalc keyboard and power controller: an STM32 on the mainboard
 * that scans the keyboard and owns the battery and charger (through the
 * AXP2101), both backlights, the amplifier enable and the power switch. It is
 * the only way the host reaches any of them. It supplies:
 *
 *   - a keyboard input source,
 *   - the battery0 provider (percentage and charging state; no voltage),
 *   - the keyboard backlight through the auxiliary backlight service,
 *   - the LCD backlight through the primary display's brightness control,
 *   - the system power-off, when named by SOLAR_OS_BOARD_POWEROFF_PROVIDER.
 *
 * It runs on PicoCalc power only. When the mainboard is unpowered (for example
 * the module is on its own USB cable) the controller does not answer: the
 * keyboard stays not-ready and polling continues until it appears. */
#define SOLAR_OS_PICOCALC_CORE_ADDRESS 0x1fU
/* Time to wait between writing a register number and reading it back. None
 * is needed: the controller builds its reply in its I2C STOP interrupt, and the
 * stock PicoMite firmware polls it with no wait. The optional gap binding adds
 * a wait for testing. */
#define SOLAR_OS_PICOCALC_CORE_DEFAULT_GAP_MS 0
#define SOLAR_OS_PICOCALC_CORE_MAX_GAP_MS 50

esp_err_t solar_os_picocalc_core_attach(
    const char *name,
    const solar_os_expansion_binding_t *bindings,
    size_t binding_count);
esp_err_t solar_os_picocalc_core_detach(const char *name);
