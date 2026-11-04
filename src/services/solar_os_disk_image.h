#pragma once

#include "solar_os_storage.h"

#define SOLAR_OS_DISK_IMAGE_MAX 4U
#define SOLAR_OS_DISK_IMAGE_MAX_BYTES 0x7ffffe00U

/* Fixed-size, 512-byte-sector raw images. A claim belongs either to a host
 * FAT mount or to one foreground worker, including while it is suspended. */
esp_err_t solar_os_disk_image_open(const char *path, const void *owner,
                                  uint32_t *handle, uint32_t *size);
esp_err_t solar_os_disk_image_read(uint32_t handle, const void *owner,
                                  uint32_t offset, void *data, size_t length);
esp_err_t solar_os_disk_image_write(uint32_t handle, const void *owner,
                                   uint32_t offset, const void *data, size_t length);
esp_err_t solar_os_disk_image_sync(uint32_t handle, const void *owner);
esp_err_t solar_os_disk_image_close(uint32_t handle, const void *owner);
esp_err_t solar_os_disk_image_close_owner(const void *owner);

esp_err_t solar_os_disk_image_mount(const char *path, const char *mount_point,
                                   unsigned partition);
esp_err_t solar_os_disk_image_unmount(const char *target);
/* Creates a new MBR disk with one FAT partition; never overwrites a file. */
esp_err_t solar_os_disk_image_create(const char *path, uint32_t size_mib);

/* A block device that backs a volume instead of a file, such as a RAM disk.
 * Offsets and lengths are byte ranges inside the device, in whole sectors. */
typedef struct {
    esp_err_t (*read)(void *context, uint32_t offset, void *data, size_t length);
    esp_err_t (*write)(void *context, uint32_t offset, const void *data, size_t length);
} solar_os_disk_image_device_t;

#define SOLAR_OS_DISK_IMAGE_DEVICE_MIN_BYTES (64U * 1024U)
#define SOLAR_OS_DISK_IMAGE_FORMAT_WORK_BYTES 4096U

/* Formats the whole device as one FAT volume (the old contents are lost) and
 * mounts it at mount_point. It is listed as block `name` of type `type_name`.
 * work is scratch memory for the formatter, at least
 * SOLAR_OS_DISK_IMAGE_FORMAT_WORK_BYTES, and is not kept. The device must stay
 * valid until the volume is unmounted, by solar_os_disk_image_detach_device()
 * or by an ordinary unmount such as the one before shutdown. */
esp_err_t solar_os_disk_image_attach_device(const char *name, const char *type_name,
                                            const solar_os_disk_image_device_t *device,
                                            void *context, uint32_t size,
                                            const char *mount_point,
                                            void *work, size_t work_size);
/* Unmounts a device volume. ESP_ERR_NOT_FOUND if it is no longer mounted. */
esp_err_t solar_os_disk_image_detach_device(const char *mount_point);
size_t solar_os_disk_image_mount_count(void);
bool solar_os_disk_image_get_mount(size_t index, solar_os_storage_mount_info_t *mount);
bool solar_os_disk_image_get_block(size_t index, solar_os_storage_block_t *block);
bool solar_os_disk_image_path_busy(const char *path);
bool solar_os_disk_image_backing_busy(const char *mount_point);
/* Serialize filesystem topology changes with image admission. Recursive so
 * storage controls can call image mount/unmount and metadata helpers. */
void solar_os_disk_image_registry_lock(void);
void solar_os_disk_image_registry_unlock(void);
