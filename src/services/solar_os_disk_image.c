#include "solar_os_disk_image.h"

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/lock.h>
#include <sys/stat.h>
#include <unistd.h>

#include "diskio_impl.h"
#include "esp_vfs_fat.h"
#include "ff.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    dev_t device;
    ino_t inode;
    uint32_t cluster;
    uint8_t volume;
} image_identity_t;

typedef struct {
    uint32_t handle, size;
    const void *owner;
    int fd;
    char path[SOLAR_OS_STORAGE_PATH_MAX];
    char backing[SOLAR_OS_STORAGE_MOUNT_POINT_MAX];
    char mount[SOLAR_OS_STORAGE_MOUNT_POINT_MAX];
    BYTE volume;
    unsigned partition;
    bool registered, mounted;
    _lock_t io_lock;
    image_identity_t identity;
    /* Device-backed volumes have no file: fd is -1 and path is empty. */
    const solar_os_disk_image_device_t *device;
    void *device_context;
    char name[SOLAR_OS_STORAGE_BLOCK_NAME_MAX];
    char type_name[SOLAR_OS_STORAGE_TYPE_NAME_MAX];
} disk_image_t;

static disk_image_t images[SOLAR_OS_DISK_IMAGE_MAX];
static disk_image_t *drives[FF_VOLUMES];
static _lock_t images_lock;
static uint32_t next_handle;
static const char mount_owner;

void solar_os_disk_image_registry_lock(void) { _lock_acquire_recursive(&images_lock); }
void solar_os_disk_image_registry_unlock(void) { _lock_release_recursive(&images_lock); }

static bool path_under(const char *path, const char *base)
{
    size_t n = strlen(base);
    return n && !strncasecmp(path, base, n) &&
        (!strcmp(base, "/") || path[n] == '\0' || path[n] == '/');
}

static esp_err_t normalize(const char *path, char *out, size_t size)
{
    if (!path || path[0] != '/') return ESP_ERR_INVALID_ARG;
    return solar_os_storage_normalize_path(path, out, size);
}

static disk_image_t *find_handle(uint32_t handle, const void *owner)
{
    for (unsigned i = 0; i < SOLAR_OS_DISK_IMAGE_MAX; ++i)
        if (handle && images[i].handle == handle && images[i].owner == owner) return &images[i];
    return NULL;
}

static esp_err_t file_identity(const char *path, int fd, image_identity_t *identity)
{
    memset(identity, 0, sizeof(*identity));
    identity->volume = UINT8_MAX;
    struct stat st = {0};
    if ((fd >= 0 ? fstat(fd, &st) : stat(path, &st)) || !S_ISREG(st.st_mode)) return ESP_ERR_NOT_FOUND;
    identity->device = st.st_dev; identity->inode = st.st_ino;
    if (st.st_ino) return ESP_OK;
    /* ESP-IDF FAT VFS has no inode numbers. The starting cluster identifies
     * a nonempty backing file across case variants and DOS 8.3 aliases. */
    char backing[SOLAR_OS_STORAGE_MOUNT_POINT_MAX];
    if (solar_os_storage_path_mount_point(path, backing, sizeof(backing)) != ESP_OK) return ESP_FAIL;
    for (size_t i = 0; i < solar_os_storage_block_count(); ++i) {
        solar_os_storage_block_t block;
        if (!solar_os_storage_get_block(i, &block) || !block.mounted ||
            strcmp(backing, block.mount_point) || block.logical_volume == UINT8_MAX) continue;
        const char *tail = path + strlen(backing);
        while (*tail == '/') ++tail;
        char fat_path[SOLAR_OS_STORAGE_PATH_MAX + 3];
        int written = snprintf(fat_path, sizeof(fat_path), "%u:/%s", block.logical_volume, tail);
        if (written < 0 || (size_t)written >= sizeof(fat_path))
            return ESP_ERR_INVALID_SIZE;
        /* FIL includes a buffer up to FF_MAX_SS (4 KiB with wear levelling).
         * Keep it off callers' smaller background-job stacks. */
        FIL *file = calloc(1, sizeof(*file));
        if (!file) return ESP_ERR_NO_MEM;
        esp_err_t err = ESP_FAIL;
        if (f_open(file, fat_path, FA_READ) == FR_OK) {
            identity->cluster = file->obj.sclust; identity->volume = block.logical_volume;
            err = f_close(file) == FR_OK ? ESP_OK : ESP_FAIL;
        }
        free(file);
        return err;
    }
    return ESP_OK; /* RAMFS uses its canonical path rather than a FAT cluster. */
}

static bool same_file(const image_identity_t *a, const image_identity_t *b)
{
    return (a->inode && b->inode && a->inode == b->inode && a->device == b->device) ||
        (a->cluster && b->cluster && a->cluster == b->cluster && a->volume == b->volume);
}

/* Caller holds images_lock. Image-on-image mounts are deliberately excluded:
 * the backing filesystem must not re-enter the same FatFS volume or create a
 * volume lock cycle. Ordinary files on SD, flash and RAMFS are supported. */
static esp_err_t open_locked(const char *path, const void *owner, int flags, disk_image_t **out)
{
    char resolved[SOLAR_OS_STORAGE_PATH_MAX], backing[SOLAR_OS_STORAGE_MOUNT_POINT_MAX];
    esp_err_t err = normalize(path, resolved, sizeof(resolved));
    if (err == ESP_OK) err = solar_os_storage_path_mount_point(resolved, backing, sizeof(backing));
    if (err != ESP_OK) return err;
    disk_image_t *slot = NULL;
    for (unsigned i = 0; i < SOLAR_OS_DISK_IMAGE_MAX; ++i) {
        disk_image_t *image = &images[i];
        if (!image->handle) { if (!slot) slot = image; continue; }
        if (!strcasecmp(image->path, resolved) ||
            (image->mounted && path_under(resolved, image->mount))) return ESP_ERR_INVALID_STATE;
    }
    if (!slot) return ESP_ERR_NO_MEM;
    int fd = open(resolved, flags, 0666);
    if (fd < 0) return errno == ENOENT ? ESP_ERR_NOT_FOUND :
        errno == EEXIST ? ESP_ERR_INVALID_STATE : ESP_FAIL;
    struct stat st;
    if (fstat(fd, &st) || !S_ISREG(st.st_mode) || st.st_size < 0 ||
        (uint64_t)st.st_size > SOLAR_OS_DISK_IMAGE_MAX_BYTES ||
        (!(flags & O_CREAT) && (!st.st_size || st.st_size % 512))) {
        close(fd); return ESP_ERR_INVALID_SIZE;
    }
    image_identity_t identity = {0};
    if (!(flags & O_CREAT)) {
        err = file_identity(resolved, fd, &identity);
        if (err != ESP_OK) { close(fd); return err; }
        for (unsigned i = 0; i < SOLAR_OS_DISK_IMAGE_MAX; ++i)
            if (images[i].handle && same_file(&identity, &images[i].identity)) {
                close(fd); return ESP_ERR_INVALID_STATE;
            }
    }
    /* Do not reset io_lock: its lifetime is the static slot's lifetime. */
    slot->fd = fd; slot->size = (uint32_t)st.st_size; slot->owner = owner;
    slot->identity = identity;
    if (++next_handle == 0) ++next_handle;
    slot->handle = next_handle;
    strlcpy(slot->path, resolved, sizeof(slot->path));
    strlcpy(slot->backing, backing, sizeof(slot->backing));
    *out = slot;
    return ESP_OK;
}

static esp_err_t transfer(disk_image_t *image, uint32_t offset, void *data, size_t length, bool write_data)
{
    if ((!data && length) || length > SOLAR_OS_STORAGE_READ_MAX_BYTES ||
        offset > image->size || length > image->size - offset) return ESP_ERR_INVALID_SIZE;
    _lock_acquire(&image->io_lock);
    esp_err_t err = ESP_OK;
    if (image->device) {
        if (length) err = write_data ?
            image->device->write(image->device_context, offset, data, length) :
            image->device->read(image->device_context, offset, data, length);
    } else {
        if (lseek(image->fd, offset, SEEK_SET) != (off_t)offset) err = ESP_FAIL;
        for (size_t done = 0; err == ESP_OK && done < length;) {
            ssize_t n = write_data ? write(image->fd, (const uint8_t *)data + done, length - done) :
                read(image->fd, (uint8_t *)data + done, length - done);
            if (n < 0 && errno == EINTR) continue;
            if (n <= 0) err = ESP_FAIL; else done += (size_t)n;
        }
    }
    _lock_release(&image->io_lock);
    return err;
}

static esp_err_t sync_image(disk_image_t *image)
{
    if (image->device) return ESP_OK; /* Device writes complete before returning. */
    _lock_acquire(&image->io_lock);
    esp_err_t err = fsync(image->fd) == 0 ? ESP_OK : ESP_FAIL;
    _lock_release(&image->io_lock);
    return err;
}

static esp_err_t close_locked(disk_image_t *image)
{
    esp_err_t err = sync_image(image);
    if (!image->device && close(image->fd) && err == ESP_OK) err = ESP_FAIL;
    image->handle = 0; image->owner = NULL; image->fd = -1;
    image->mounted = false; image->registered = false; image->mount[0] = '\0';
    image->device = NULL; image->device_context = NULL;
    image->name[0] = '\0'; image->type_name[0] = '\0';
    return err;
}

esp_err_t solar_os_disk_image_open(const char *path, const void *owner, uint32_t *handle, uint32_t *size)
{
    if (!owner || !handle || !size) return ESP_ERR_INVALID_ARG;
    *handle = 0;
    _lock_acquire_recursive(&images_lock);
    disk_image_t *image = NULL;
    esp_err_t err = open_locked(path, owner, O_RDWR, &image);
    if (err == ESP_OK) { *handle = image->handle; *size = image->size; }
    _lock_release_recursive(&images_lock);
    return err;
}

static esp_err_t handle_transfer(uint32_t handle, const void *owner, uint32_t offset,
                                void *data, size_t length, bool write_data)
{
    _lock_acquire_recursive(&images_lock);
    disk_image_t *image = find_handle(handle, owner);
    esp_err_t err = image ? transfer(image, offset, data, length, write_data) : ESP_ERR_INVALID_STATE;
    /* Guest writes reach the backing filesystem before ATA/floppy success. */
    if (err == ESP_OK && write_data) err = sync_image(image);
    _lock_release_recursive(&images_lock);
    return err;
}

esp_err_t solar_os_disk_image_read(uint32_t handle, const void *owner, uint32_t offset, void *data, size_t length)
{ return handle_transfer(handle, owner, offset, data, length, false); }
esp_err_t solar_os_disk_image_write(uint32_t handle, const void *owner, uint32_t offset, const void *data, size_t length)
{ return handle_transfer(handle, owner, offset, (void *)data, length, true); }

esp_err_t solar_os_disk_image_sync(uint32_t handle, const void *owner)
{
    _lock_acquire_recursive(&images_lock);
    disk_image_t *image = find_handle(handle, owner);
    esp_err_t err = image ? sync_image(image) : ESP_ERR_INVALID_STATE;
    _lock_release_recursive(&images_lock);
    return err;
}

esp_err_t solar_os_disk_image_close(uint32_t handle, const void *owner)
{
    _lock_acquire_recursive(&images_lock);
    disk_image_t *image = find_handle(handle, owner);
    esp_err_t err = image && !image->mounted ? close_locked(image) : ESP_ERR_INVALID_STATE;
    _lock_release_recursive(&images_lock);
    return err;
}

esp_err_t solar_os_disk_image_close_owner(const void *owner)
{
    if (!owner || owner == &mount_owner) return ESP_ERR_INVALID_ARG;
    _lock_acquire_recursive(&images_lock);
    esp_err_t err = ESP_OK;
    for (unsigned i = 0; i < SOLAR_OS_DISK_IMAGE_MAX; ++i) {
        if (!images[i].handle || images[i].owner != owner) continue;
        esp_err_t e = close_locked(&images[i]);
        if (e != ESP_OK) err = e;
    }
    _lock_release_recursive(&images_lock);
    return err;
}

static DSTATUS image_disk_status(BYTE drive)
{ return drive < FF_VOLUMES && drives[drive] ? 0 : STA_NOINIT; }
static DRESULT disk_transfer(BYTE drive, BYTE *data, LBA_t sector, UINT count, bool write_data)
{
    disk_image_t *image = drive < FF_VOLUMES ? drives[drive] : NULL;
    if (!image || !count || sector >= image->size / 512U || count > image->size / 512U - sector)
        return RES_PARERR;
    while (count) {
        UINT n = count > 128U ? 128U : count;
        if (transfer(image, (uint32_t)sector * 512U, data, (size_t)n * 512U, write_data) != ESP_OK)
            return RES_ERROR;
        sector += n; count -= n; data += n * 512U;
    }
    return RES_OK;
}
static DRESULT image_disk_read(BYTE drive, BYTE *data, LBA_t sector, UINT count)
{ return disk_transfer(drive, data, sector, count, false); }
static DRESULT image_disk_write(BYTE drive, const BYTE *data, LBA_t sector, UINT count)
{ return disk_transfer(drive, (BYTE *)data, sector, count, true); }
static DRESULT image_disk_ioctl(BYTE drive, BYTE command, void *data)
{
    disk_image_t *image = drive < FF_VOLUMES ? drives[drive] : NULL;
    if (!image) return RES_PARERR;
    if (command == CTRL_SYNC) return sync_image(image) == ESP_OK ? RES_OK : RES_ERROR;
    if (!data) return RES_PARERR;
    switch (command) {
    case GET_SECTOR_COUNT: *(LBA_t *)data = image->size / 512U; return RES_OK;
    case GET_SECTOR_SIZE: *(WORD *)data = 512; return RES_OK;
    case GET_BLOCK_SIZE: *(DWORD *)data = 1; return RES_OK;
    default: return RES_PARERR;
    }
}
static const ff_diskio_impl_t image_diskio = {
    .init = image_disk_status, .status = image_disk_status,
    .read = image_disk_read, .write = image_disk_write, .ioctl = image_disk_ioctl,
};

static esp_err_t register_drive(disk_image_t *image, unsigned partition)
{
    BYTE drive;
    if (ff_diskio_get_drive(&drive) != ESP_OK) return ESP_ERR_NO_MEM;
    image->volume = drive; image->partition = partition;
    drives[drive] = image;
    ff_diskio_register(drive, &image_diskio);
    VolToPart[drive].pd = drive; VolToPart[drive].pt = partition;
    image->registered = true;
    return ESP_OK;
}

static void unregister_drive(disk_image_t *image)
{
    if (!image->registered) return;
    ff_diskio_unregister(image->volume);
    drives[image->volume] = NULL;
    VolToPart[image->volume].pd = image->volume; VolToPart[image->volume].pt = 0;
    image->registered = false;
}

/* Caller holds images_lock. Rejects mounts that would hide existing mounts or
 * their backing files. */
static esp_err_t check_mount_point_locked(const char *mount)
{
    for (size_t i = 0; i < solar_os_storage_mount_count(); ++i) {
        solar_os_storage_mount_info_t info;
        if (solar_os_storage_get_mount(i, &info) && (path_under(info.mount_point, mount) ||
            (info.type == SOLAR_OS_STORAGE_MOUNT_IMAGE && path_under(mount, info.mount_point))))
            return ESP_ERR_INVALID_STATE;
    }
    for (unsigned i = 0; i < SOLAR_OS_DISK_IMAGE_MAX; ++i)
        if (images[i].handle && path_under(images[i].path, mount)) return ESP_ERR_INVALID_STATE;
    return ESP_OK;
}

esp_err_t solar_os_disk_image_mount(const char *path, const char *mount_point, unsigned partition)
{
    if (!mount_point || mount_point[0] != '/' || !strcmp(mount_point, "/") ||
        strlen(mount_point) >= sizeof(images[0].mount) || partition > 4) return ESP_ERR_INVALID_ARG;
    char mount[SOLAR_OS_STORAGE_MOUNT_POINT_MAX];
    esp_err_t err = solar_os_storage_join_path("/", mount_point, mount, sizeof(mount));
    if (err != ESP_OK) return err;
    if (!strcmp(mount, "/")) return ESP_ERR_INVALID_ARG;
    _lock_acquire_recursive(&images_lock);
    err = check_mount_point_locked(mount);
    disk_image_t *image = NULL;
    if (err == ESP_OK) err = open_locked(path, &mount_owner, O_RDWR, &image);
    if (err == ESP_OK && path_under(image->path, mount)) err = ESP_ERR_INVALID_STATE;
    if (err == ESP_OK) err = register_drive(image, partition);
    FATFS *fs = NULL;
    bool vfs_registered = false;
    char drive[3] = {0};
    if (err == ESP_OK) {
        drive[0] = '0' + image->volume; drive[1] = ':';
        esp_vfs_fat_conf_t conf = {.base_path = mount, .fat_drive = drive, .max_files = 8};
        err = esp_vfs_fat_register_cfg(&conf, &fs);
        vfs_registered = err == ESP_OK;
        if (err == ESP_OK) {
            FRESULT result = f_mount(fs, drive, 1);
            err = result == FR_OK ? ESP_OK : result == FR_NO_FILESYSTEM ? ESP_ERR_NOT_SUPPORTED : ESP_FAIL;
        }
    }
    if (err == ESP_OK) {
        strlcpy(image->mount, mount, sizeof(image->mount)); image->mounted = true;
    } else if (image) {
        if (vfs_registered) { (void)f_mount(NULL, drive, 0); (void)esp_vfs_fat_unregister_path(mount); }
        unregister_drive(image); (void)close_locked(image);
    }
    _lock_release_recursive(&images_lock);
    return err;
}

/* Caller holds images_lock. */
static esp_err_t unmount_locked(disk_image_t *image)
{
    esp_err_t err = sync_image(image);
    if (err != ESP_OK) return err; /* Keep the mount and claim for a retry. */
    char drive[3] = {'0' + image->volume, ':', 0};
    if (f_mount(NULL, drive, 0) != FR_OK) return ESP_FAIL;
    err = esp_vfs_fat_unregister_path(image->mount);
    unregister_drive(image);
    esp_err_t close_err = close_locked(image);
    return err == ESP_OK ? close_err : err;
}

esp_err_t solar_os_disk_image_unmount(const char *target)
{
    if (!target) return ESP_ERR_NOT_FOUND;
    _lock_acquire_recursive(&images_lock);
    esp_err_t err = ESP_ERR_NOT_FOUND;
    for (unsigned i = 0; i < SOLAR_OS_DISK_IMAGE_MAX; ++i) {
        disk_image_t *image = &images[i];
        char name[12]; snprintf(name, sizeof(name), "img%u", i);
        const char *block_name = image->device ? image->name : name;
        if (!image->mounted || (strcmp(target, image->mount) && strcmp(target, block_name) &&
            (image->device || strcasecmp(target, image->path)))) continue;
        err = unmount_locked(image);
        break;
    }
    _lock_release_recursive(&images_lock);
    return err;
}

esp_err_t solar_os_disk_image_attach_device(const char *name, const char *type_name,
                                            const solar_os_disk_image_device_t *device,
                                            void *context, uint32_t size,
                                            const char *mount_point,
                                            void *work, size_t work_size)
{
    if (!name || !name[0] || strlen(name) >= sizeof(images[0].name) ||
        !type_name || !type_name[0] || strlen(type_name) >= sizeof(images[0].type_name) ||
        !device || !device->read || !device->write || !work ||
        work_size < SOLAR_OS_DISK_IMAGE_FORMAT_WORK_BYTES || !mount_point ||
        mount_point[0] != '/' || strlen(mount_point) >= sizeof(images[0].mount))
        return ESP_ERR_INVALID_ARG;
    if (size < SOLAR_OS_DISK_IMAGE_DEVICE_MIN_BYTES || size % 512U ||
        size > SOLAR_OS_DISK_IMAGE_MAX_BYTES) return ESP_ERR_INVALID_SIZE;
    char mount[SOLAR_OS_STORAGE_MOUNT_POINT_MAX];
    esp_err_t err = solar_os_storage_join_path("/", mount_point, mount, sizeof(mount));
    if (err != ESP_OK) return err;
    if (!strcmp(mount, "/")) return ESP_ERR_INVALID_ARG;
    _lock_acquire_recursive(&images_lock);
    err = check_mount_point_locked(mount);
    disk_image_t *image = NULL;
    for (unsigned i = 0; err == ESP_OK && i < SOLAR_OS_DISK_IMAGE_MAX; ++i) {
        if (images[i].handle && images[i].device && !strcmp(images[i].name, name)) {
            err = ESP_ERR_INVALID_STATE;
        } else if (!images[i].handle && !image) {
            image = &images[i];
        }
    }
    if (err == ESP_OK && !image) err = ESP_ERR_NO_MEM;
    if (err != ESP_OK) image = NULL;
    if (image) {
        /* Do not reset io_lock: its lifetime is the static slot's lifetime. */
        image->fd = -1; image->size = size; image->owner = &mount_owner;
        memset(&image->identity, 0, sizeof(image->identity));
        image->path[0] = '\0'; image->backing[0] = '\0';
        image->device = device; image->device_context = context;
        strlcpy(image->name, name, sizeof(image->name));
        strlcpy(image->type_name, type_name, sizeof(image->type_name));
        if (++next_handle == 0) ++next_handle;
        image->handle = next_handle;
        err = register_drive(image, 0);
    }
    FATFS *fs = NULL;
    bool vfs_registered = false;
    char drive[3] = {0};
    if (err == ESP_OK) {
        drive[0] = '0' + image->volume; drive[1] = ':';
        /* No partition table, and one FAT: the volume lives only until the
         * device loses power. */
        const MKFS_PARM options = {.fmt = FM_ANY | FM_SFD, .n_fat = 1};
        if (f_mkfs(drive, &options, work, (UINT)work_size) != FR_OK) err = ESP_FAIL;
    }
    if (err == ESP_OK) {
        esp_vfs_fat_conf_t conf = {.base_path = mount, .fat_drive = drive, .max_files = 8};
        err = esp_vfs_fat_register_cfg(&conf, &fs);
        vfs_registered = err == ESP_OK;
        if (err == ESP_OK && f_mount(fs, drive, 1) != FR_OK) err = ESP_FAIL;
    }
    if (err == ESP_OK) {
        strlcpy(image->mount, mount, sizeof(image->mount)); image->mounted = true;
    } else if (image) {
        if (vfs_registered) { (void)f_mount(NULL, drive, 0); (void)esp_vfs_fat_unregister_path(mount); }
        unregister_drive(image); (void)close_locked(image);
    }
    _lock_release_recursive(&images_lock);
    return err;
}

esp_err_t solar_os_disk_image_detach_device(const char *mount_point)
{
    if (!mount_point) return ESP_ERR_INVALID_ARG;
    char mount[SOLAR_OS_STORAGE_MOUNT_POINT_MAX];
    if (solar_os_storage_join_path("/", mount_point, mount, sizeof(mount)) != ESP_OK)
        return ESP_ERR_NOT_FOUND;
    _lock_acquire_recursive(&images_lock);
    esp_err_t err = ESP_ERR_NOT_FOUND;
    for (unsigned i = 0; i < SOLAR_OS_DISK_IMAGE_MAX; ++i) {
        if (!images[i].mounted || !images[i].device || strcmp(images[i].mount, mount)) continue;
        err = unmount_locked(&images[i]);
        break;
    }
    _lock_release_recursive(&images_lock);
    return err;
}

static void set_chs(uint8_t *out, uint32_t lba, unsigned heads)
{
    unsigned cylinder = lba / (heads * 63U);
    if (cylinder > 1023) { out[0] = heads - 1; out[1] = 0xff; out[2] = 0xff; return; }
    out[0] = lba / 63U % heads;
    out[1] = (lba % 63U + 1U) | ((cylinder >> 2) & 0xc0);
    out[2] = cylinder;
}

static uint32_t read_le32(const uint8_t *p)
{ return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

static esp_err_t set_dos_geometry(disk_image_t *image, uint8_t *sector)
{
    /* Match conventional BIOS LBA translation rather than FatFS's generic
     * 255-head BPB, which DOS would otherwise use for incorrect CHS reads. */
    unsigned h = image->size / 512U / 63U / 1024U;
    unsigned heads = h > 128 ? 255 : h > 64 ? 128 : h > 32 ? 64 : h > 16 ? 32 : 16;
    esp_err_t err = transfer(image, 0, sector, 512, false);
    if (err != ESP_OK) return err;
    uint8_t *partition = sector + 446;
    uint32_t start = read_le32(partition + 8), count = read_le32(partition + 12);
    if (!start || !count || start >= image->size / 512U || count > image->size / 512U - start) return ESP_FAIL;
    partition[0] = 0x80;
    set_chs(partition + 1, start, heads); set_chs(partition + 5, start + count - 1, heads);
    err = transfer(image, 0, sector, 512, true);
    if (err == ESP_OK) err = transfer(image, start * 512U, sector, 512, false);
    if (err == ESP_OK) {
        sector[24] = 63; sector[25] = 0; sector[26] = heads; sector[27] = 0; sector[36] = 0x80;
        err = transfer(image, start * 512U, sector, 512, true);
    }
    return err;
}

esp_err_t solar_os_disk_image_create(const char *path, uint32_t size_mib)
{
    if (!size_mib || size_mib > SOLAR_OS_DISK_IMAGE_MAX_BYTES / (1024U * 1024U)) return ESP_ERR_INVALID_SIZE;
    void *work = calloc(1, 8192);
    if (!work) return ESP_ERR_NO_MEM;
    _lock_acquire_recursive(&images_lock);
    disk_image_t *image = NULL;
    esp_err_t err = open_locked(path, &mount_owner, O_RDWR | O_CREAT | O_EXCL, &image);
    if (err == ESP_OK) {
        image->size = size_mib * 1024U * 1024U;
        for (uint32_t offset = 0; err == ESP_OK && offset < image->size; offset += 8192) {
            err = transfer(image, offset, work, 8192, true);
            vTaskDelay(1);
        }
        if (err == ESP_OK) err = register_drive(image, 1);
        if (err == ESP_OK) {
            char drive[3] = {'0' + image->volume, ':', 0};
            const LBA_t partitions[4] = {100, 0, 0, 0};
            const MKFS_PARM options = {.fmt = FM_FAT, .n_fat = 2};
            if (f_fdisk(image->volume, partitions, work) != FR_OK ||
                f_mkfs(drive, &options, work, 8192) != FR_OK) err = ESP_FAIL;
            if (err == ESP_OK) err = set_dos_geometry(image, work);
        }
        unregister_drive(image);
        esp_err_t close_err = close_locked(image);
        if (err == ESP_OK) err = close_err;
        if (err != ESP_OK) (void)unlink(image->path);
    }
    _lock_release_recursive(&images_lock);
    free(work);
    return err;
}

/* Read-only metadata queries also run inside mount admission. Use a recursive
 * registry lock; diskio callbacks use only the per-image I/O lock. */
size_t solar_os_disk_image_mount_count(void)
{
    _lock_acquire_recursive(&images_lock);
    size_t count = 0;
    for (unsigned i = 0; i < SOLAR_OS_DISK_IMAGE_MAX; ++i) count += images[i].mounted;
    _lock_release_recursive(&images_lock);
    return count;
}

bool solar_os_disk_image_get_block(size_t index, solar_os_storage_block_t *block)
{
    if (!block) return false;
    _lock_acquire_recursive(&images_lock);
    for (unsigned i = 0; i < SOLAR_OS_DISK_IMAGE_MAX; ++i) {
        disk_image_t *image = &images[i];
        if (!image->mounted) continue;
        if (index--) continue;
        memset(block, 0, sizeof(*block));
        if (image->device) strlcpy(block->name, image->name, sizeof(block->name));
        else snprintf(block->name, sizeof(block->name), "img%u", i);
        block->type = SOLAR_OS_STORAGE_BLOCK_DISK; block->mounted = true; block->mountable = true;
        block->logical_volume = image->volume; block->partition_number = image->partition;
        block->sector_size = 512; block->sector_count = image->size / 512U; block->size_bytes = image->size;
        strlcpy(block->fs, "FAT", sizeof(block->fs));
        strlcpy(block->type_name, image->device ? image->type_name : "image", sizeof(block->type_name));
        strlcpy(block->mount_point, image->mount, sizeof(block->mount_point));
        _lock_release_recursive(&images_lock);
        return true;
    }
    _lock_release_recursive(&images_lock);
    return false;
}

bool solar_os_disk_image_get_mount(size_t index, solar_os_storage_mount_info_t *mount)
{
    solar_os_storage_block_t block;
    if (!mount || !solar_os_disk_image_get_block(index, &block)) return false;
    memset(mount, 0, sizeof(*mount));
    mount->type = SOLAR_OS_STORAGE_MOUNT_IMAGE;
    strlcpy(mount->name, block.name, sizeof(mount->name));
    strlcpy(mount->mount_point, block.mount_point, sizeof(mount->mount_point));
    return true;
}

bool solar_os_disk_image_path_busy(const char *path)
{
    char resolved[SOLAR_OS_STORAGE_PATH_MAX];
    if (normalize(path, resolved, sizeof(resolved)) != ESP_OK) return false;
    _lock_acquire_recursive(&images_lock);
    bool busy = false, active = false;
    for (unsigned i = 0; i < SOLAR_OS_DISK_IMAGE_MAX; ++i)
        if (images[i].handle) {
            active = true;
            if (path_under(images[i].path, resolved)) { busy = true; break; }
        }
    if (!busy && active) {
        image_identity_t identity;
        if (file_identity(resolved, -1, &identity) == ESP_OK)
            for (unsigned i = 0; i < SOLAR_OS_DISK_IMAGE_MAX; ++i)
                if (images[i].handle && same_file(&identity, &images[i].identity)) { busy = true; break; }
    }
    _lock_release_recursive(&images_lock);
    return busy;
}

bool solar_os_disk_image_backing_busy(const char *mount_point)
{
    if (!mount_point) return false;
    _lock_acquire_recursive(&images_lock);
    bool busy = false;
    for (unsigned i = 0; i < SOLAR_OS_DISK_IMAGE_MAX; ++i)
        if (images[i].handle && !strcmp(images[i].backing, mount_point)) { busy = true; break; }
    _lock_release_recursive(&images_lock);
    return busy;
}
