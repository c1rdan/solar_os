#include "solar_os_sdmmc.h"

#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sd_card.h"
#include "solar_os_board_caps.h"
#include "solar_os_card_detect.h"
#include "solar_os_storage.h"
#include "solar_os_task.h"

/* Card-detect polling. The switch is slow and mechanical, so a 250 ms period
 * with three agreeing samples (about 0.75 s) is ample and ignores bounce. */
#define SDMMC_DETECT_POLL_MS 250U
#define SDMMC_DETECT_STABLE_SAMPLES 3U
#define SDMMC_DETECT_TASK_STACK 3072U
#define SDMMC_DETECT_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define SDMMC_DETECT_MOUNT_ATTEMPTS 3U
#define SDMMC_DETECT_MOUNT_RETRY_MS 500U

typedef struct {
    bool active;
    char name[SOLAR_OS_EXPANSION_DEVICE_NAME_MAX];
    /* Optional card-detect switch; cd_pin < 0 when the slot has none. */
    int cd_pin;
    int cd_present_level;
    TaskHandle_t detect_task;
    volatile bool detect_stop;
    volatile bool detect_done;
} solar_os_sdmmc_device_t;

static const char *TAG = "sdmmc";
static solar_os_sdmmc_device_t sdmmc;

static esp_err_t parse_bindings(const solar_os_expansion_binding_t *bindings,
                                size_t binding_count,
                                int pins[6], int *power, int *active,
                                int *cd, int *cd_present_level)
{
    static const char *const roles[] = {"clk", "cmd", "d0", "d1", "d2", "d3"};
    bool present[6] = {false};
    bool have_power = false, have_active = false;
    bool have_cd = false, have_cd_active = false;
    if (bindings == NULL || pins == NULL || power == NULL || active == NULL ||
        cd == NULL || cd_present_level == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *power = -1;
    *active = 1;
    *cd = -1;
    /* Card-detect switches ground the line when a card is seated. */
    *cd_present_level = 0;
    for (size_t i = 0; i < 6; i++) {
        pins[i] = -1;
    }

    for (size_t i = 0; i < binding_count; i++) {
        if (bindings[i].kind == SOLAR_OS_EXPANSION_BINDING_GPIO && !have_power &&
            strcmp(bindings[i].role, "power") == 0) {
            *power = bindings[i].value;
            have_power = true;
            continue;
        }
        if (bindings[i].kind == SOLAR_OS_EXPANSION_BINDING_PARAMETER && !have_active &&
            strcmp(bindings[i].role, "active") == 0 &&
            bindings[i].value >= 0 && bindings[i].value <= 1) {
            *active = bindings[i].value;
            have_active = true;
            continue;
        }
        if (bindings[i].kind == SOLAR_OS_EXPANSION_BINDING_GPIO && !have_cd &&
            strcmp(bindings[i].role, "cd") == 0) {
            *cd = bindings[i].value;
            have_cd = true;
            continue;
        }
        if (bindings[i].kind == SOLAR_OS_EXPANSION_BINDING_PARAMETER && !have_cd_active &&
            strcmp(bindings[i].role, "cd_active") == 0 &&
            bindings[i].value >= 0 && bindings[i].value <= 1) {
            *cd_present_level = bindings[i].value;
            have_cd_active = true;
            continue;
        }
        if (bindings[i].kind != SOLAR_OS_EXPANSION_BINDING_GPIO) {
            return ESP_ERR_INVALID_ARG;
        }
        size_t role = 0;
        while (role < 6 && strcmp(bindings[i].role, roles[role]) != 0) {
            role++;
        }
        if (role == 6 || present[role]) {
            return ESP_ERR_INVALID_ARG;
        }
        pins[role] = bindings[i].value;
        present[role] = true;
    }

    if ((!have_power && have_active) || (!have_cd && have_cd_active) ||
        !present[0] || !present[1] || !present[2] ||
        (present[3] != present[4]) || (present[3] != present[5])) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static bool card_present(const solar_os_sdmmc_device_t *device)
{
    return solar_os_card_detect_present_from_level(gpio_get_level((gpio_num_t)device->cd_pin),
                                                   device->cd_present_level);
}

static void detect_mount(void)
{
    esp_err_t err = ESP_FAIL;
    /* A freshly seated card can take a moment to answer. */
    for (unsigned attempt = 0U; attempt < SDMMC_DETECT_MOUNT_ATTEMPTS; attempt++) {
        err = solar_os_storage_mount();
        if (err == ESP_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(SDMMC_DETECT_MOUNT_RETRY_MS));
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "card inserted and mounted");
    } else {
        ESP_LOGW(TAG, "card inserted but mount failed: %s", esp_err_to_name(err));
    }
}

static void detect_worker(void *arg)
{
    solar_os_sdmmc_device_t *device = arg;
    solar_os_card_detect_t detect;
    solar_os_card_detect_init(&detect, card_present(device), SDMMC_DETECT_STABLE_SAMPLES);

    while (!device->detect_stop) {
        vTaskDelay(pdMS_TO_TICKS(SDMMC_DETECT_POLL_MS));
        switch (solar_os_card_detect_sample(&detect, card_present(device))) {
        case SOLAR_OS_CARD_DETECT_INSERTED:
            detect_mount();
            break;
        case SOLAR_OS_CARD_DETECT_REMOVED: {
            /* The card is already gone: unmount without expecting a clean sync. */
            const esp_err_t err = solar_os_storage_unmount();
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "card removed and unmounted");
            } else {
                ESP_LOGW(TAG, "card removed; unmount failed: %s", esp_err_to_name(err));
            }
            break;
        }
        case SOLAR_OS_CARD_DETECT_NONE:
        default:
            break;
        }
    }
    device->detect_done = true;
    solar_os_task_delete_internal(NULL);
}

static esp_err_t start_detect(solar_os_sdmmc_device_t *device)
{
    if (device->cd_pin < 0) {
        return ESP_OK;
    }
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << device->cd_pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&config);
    if (err != ESP_OK) {
        return err;
    }
    device->detect_stop = false;
    device->detect_done = false;
    return solar_os_task_create_pinned_internal(detect_worker,
                                                "sdmmc-detect",
                                                SDMMC_DETECT_TASK_STACK,
                                                device,
                                                SDMMC_DETECT_TASK_PRIORITY,
                                                &device->detect_task,
                                                tskNO_AFFINITY,
                                                SOLAR_OS_TASK_ROLE_BACKGROUND) == pdPASS ?
        ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t stop_detect(solar_os_sdmmc_device_t *device)
{
    if (device->detect_task == NULL) {
        return ESP_OK;
    }
    device->detect_stop = true;
    if (!solar_os_task_wait_done(device->detect_task, &device->detect_done,
                                 SOLAR_OS_TASK_STOP_WAIT_MS)) {
        return ESP_ERR_TIMEOUT;
    }
    device->detect_task = NULL;
    return ESP_OK;
}

esp_err_t solar_os_sdmmc_attach(const char *name,
                                const solar_os_expansion_binding_t *bindings,
                                size_t binding_count)
{
    int pins[6];
    int power, active;
    int cd, cd_present_level;
    if (name == NULL || name[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    if (sdmmc.active) {
        return strcmp(sdmmc.name, name) == 0 ?
            solar_os_storage_mount() : ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = parse_bindings(bindings, binding_count, pins, &power, &active,
                                   &cd, &cd_present_level);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = sd_card_configure_sdmmc(pins[0], pins[1], pins[2], pins[3], pins[4], pins[5]);
    if (ret != ESP_OK) {
        return ret;
    }

    if (power >= 0) {
        ret = sd_card_configure_sdmmc_power(power, active);
        if (ret != ESP_OK) {
            (void)sd_card_clear_sdmmc_config();
            return ret;
        }
    }

    /* Built-in cards mount during normal storage initialization. */
    if (!solar_os_board_has(SOLAR_OS_BOARD_CAP_SD)) {
        ret = sd_card_init();
        if (ret != ESP_OK) {
            (void)sd_card_unmount();
            (void)sd_card_clear_sdmmc_config();
            return ret;
        }
    }

    memset(&sdmmc, 0, sizeof(sdmmc));
    sdmmc.active = true;
    sdmmc.cd_pin = cd;
    sdmmc.cd_present_level = cd_present_level;
    strlcpy(sdmmc.name, name, sizeof(sdmmc.name));
    ret = start_detect(&sdmmc);
    if (ret != ESP_OK) {
        /* The card still works without hot-plug handling. */
        ESP_LOGW(TAG, "card detect unavailable: %s", esp_err_to_name(ret));
        sdmmc.cd_pin = -1;
    }
    return ESP_OK;
}

esp_err_t solar_os_sdmmc_detach(const char *name)
{
    if (!sdmmc.active || name == NULL || strcmp(sdmmc.name, name) != 0) {
        return ESP_ERR_NOT_FOUND;
    }
    if (sd_card_has_mounts()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (stop_detect(&sdmmc) != ESP_OK) {
        return ESP_ERR_TIMEOUT;
    }

    (void)sd_card_unmount();
    const esp_err_t ret = sd_card_clear_sdmmc_config();
    if (ret == ESP_OK) {
        memset(&sdmmc, 0, sizeof(sdmmc));
    }
    return ret;
}
