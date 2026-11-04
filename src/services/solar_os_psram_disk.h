#pragma once

#include "solar_os_expansion.h"

/* A serial PSRAM chip on its own SPI host, formatted as a FAT RAM disk at
 * /psram on every boot. Its contents are lost when the board resets. */
#define SOLAR_OS_PSRAM_DISK_MOUNT_POINT "/psram"

esp_err_t solar_os_psram_disk_attach(const char *name,
                                     const solar_os_expansion_binding_t *bindings,
                                     size_t binding_count);
esp_err_t solar_os_psram_disk_detach(const char *name);
