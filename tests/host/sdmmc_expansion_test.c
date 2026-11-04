#include <assert.h>
#include <stdio.h>
#include "../../src/services/solar_os_sdmmc.c"
#include "../../src/services/solar_os_sdmmc_driver.c"

/* Card-detect: the GPIO, the polling task and the storage service are scripted. */
static unsigned gpio_configs, gpio_pin_masks[4];
static gpio_pullup_t gpio_pullups[4];
static const char *level_script;
static unsigned level_index, storage_mounts, storage_unmounts, tasks_created, tasks_deleted;
static TaskFunction_t task_function;
static void *task_argument;
static bool wait_done_result = true, task_create_fails;
static esp_err_t storage_mount_error;
esp_err_t gpio_config(const gpio_config_t *config)
{
    assert(config->mode == GPIO_MODE_INPUT && config->pull_down_en == GPIO_PULLDOWN_DISABLE);
    gpio_pin_masks[gpio_configs] = (unsigned)config->pin_bit_mask;
    gpio_pullups[gpio_configs++] = config->pull_up_en;
    return ESP_OK;
}
int gpio_get_level(gpio_num_t gpio)
{
    assert(gpio == 6);
    /* The script is read one character per sample. When it runs out the worker
     * is told to stop, and the last level is held. */
    if (level_script[level_index + 1] == '\0') sdmmc.detect_stop = true;
    const char level = level_script[level_index];
    if (level_script[level_index + 1] != '\0') level_index++;
    return level == '1';
}
BaseType_t solar_os_task_create_pinned_internal(TaskFunction_t task, const char *name,
    uint32_t stack, void *parameters, UBaseType_t priority, TaskHandle_t *handle,
    BaseType_t core, solar_os_task_role_t role)
{
    (void)name; (void)stack; (void)priority; (void)core;
    assert(role == SOLAR_OS_TASK_ROLE_BACKGROUND);
    if (task_create_fails) return 0;
    task_function = task; task_argument = parameters; tasks_created++;
    *handle = (TaskHandle_t)&task_function;
    return pdPASS;
}
void solar_os_task_delete_internal(TaskHandle_t task) { assert(task == NULL); tasks_deleted++; }
bool solar_os_task_wait_done(TaskHandle_t task, volatile bool *done, uint32_t timeout)
{ (void)task; assert(timeout == SOLAR_OS_TASK_STOP_WAIT_MS); if (wait_done_result) *done = true; return wait_done_result; }
TickType_t xTaskGetTickCount(void) { return 0; }
void vTaskDelay(TickType_t ticks) { (void)ticks; }
esp_err_t solar_os_storage_unmount(void) { storage_unmounts++; return ESP_OK; }

static int configured, power_configured, cleared, mounted;
static bool built_in = true, has_mounts;
static esp_err_t power_error, mount_error;
size_t strlcpy(char *dst, const char *src, size_t size)
{ size_t len=strlen(src); if (size) { size_t n=len<size-1?len:size-1; memcpy(dst,src,n); dst[n]=0; } return len; }
bool solar_os_board_has(solar_os_board_capabilities_t cap)
{ assert(cap==SOLAR_OS_BOARD_CAP_SD); return built_in; }
esp_err_t sd_card_configure_sdmmc(int clk, int cmd, int d0, int d1, int d2, int d3)
{ assert(clk==41 && cmd==42 && d0==40 && d1==-1 && d2==-1 && d3==-1); configured++; return ESP_OK; }
esp_err_t sd_card_configure_sdmmc_power(int pin, int active)
{ assert(pin==5 && active==0 && configured); power_configured++; return power_error; }
esp_err_t sd_card_clear_sdmmc_config(void) { cleared++; return ESP_OK; }
esp_err_t sd_card_init(void) { mounted++; return mount_error; }
esp_err_t solar_os_storage_mount(void)
{
    mounted++;
    if (storage_mounts++ < 100 && storage_mount_error != ESP_OK) return storage_mount_error;
    return mount_error;
}
esp_err_t sd_card_unmount(void) { return ESP_OK; }
bool sd_card_has_mounts(void) { return has_mounts; }

int main(void)
{
    solar_os_expansion_binding_t b[] = {
        {.kind=SOLAR_OS_EXPANSION_BINDING_GPIO, .role="clk", .value=41},
        {.kind=SOLAR_OS_EXPANSION_BINDING_GPIO, .role="cmd", .value=42},
        {.kind=SOLAR_OS_EXPANSION_BINDING_GPIO, .role="d0", .value=40},
        {.kind=SOLAR_OS_EXPANSION_BINDING_GPIO, .role="power", .value=5},
        {.kind=SOLAR_OS_EXPANSION_BINDING_PARAMETER, .role="active", .value=0},
    };
    assert(solar_os_sdmmc_attach("storage0",b,5)==ESP_OK && power_configured==1 && mounted==0);
    assert(solar_os_sdmmc_attach("other",b,5)==ESP_ERR_INVALID_ARG);
    mount_error=ESP_ERR_TIMEOUT;
    assert(solar_os_sdmmc_attach("storage0",b,5)==ESP_ERR_TIMEOUT && configured==1);
    mount_error=ESP_OK;
    assert(solar_os_sdmmc_attach("storage0",b,5)==ESP_OK && configured==1);
    mounted=0;
    has_mounts=true; assert(solar_os_sdmmc_detach("storage0")==ESP_ERR_INVALID_STATE);
    has_mounts=false; assert(solar_os_sdmmc_detach("storage0")==ESP_OK && cleared==1);
    power_error=ESP_FAIL;
    assert(solar_os_sdmmc_attach("storage0",b,5)==ESP_FAIL && cleared==2 && !sdmmc.active);
    power_error=ESP_OK; built_in=false; mount_error=ESP_ERR_TIMEOUT;
    assert(solar_os_sdmmc_attach("storage0",b,5)==ESP_ERR_TIMEOUT && cleared==3 && mounted==1);
    mount_error=ESP_OK;
    assert(solar_os_sdmmc_attach("storage0",b,3)==ESP_OK && power_configured==3 && mounted==2);
    assert(solar_os_sdmmc_detach("storage0")==ESP_OK);
    b[3]=b[4];
    assert(solar_os_sdmmc_attach("storage0",b,4)==ESP_ERR_INVALID_ARG);
    puts("SDMMC power-binding and rollback tests passed");

    /* ---- card-detect ---- */
    solar_os_expansion_binding_t d[] = {
        {.kind=SOLAR_OS_EXPANSION_BINDING_GPIO, .role="clk", .value=41},
        {.kind=SOLAR_OS_EXPANSION_BINDING_GPIO, .role="cmd", .value=42},
        {.kind=SOLAR_OS_EXPANSION_BINDING_GPIO, .role="d0", .value=40},
        {.kind=SOLAR_OS_EXPANSION_BINDING_GPIO, .role="cd", .value=6},
    };
    built_in = true;
    has_mounts = false;

    /* A slot with no detect switch starts nothing. */
    gpio_configs = tasks_created = 0;
    assert(solar_os_sdmmc_attach("storage0", b, 3) == ESP_OK);
    assert(sdmmc.cd_pin < 0 && gpio_configs == 0 && tasks_created == 0 && !sdmmc.detect_task);
    assert(solar_os_sdmmc_detach("storage0") == ESP_OK);

    /* cd_active without cd, or with a bad level, is rejected. */
    solar_os_expansion_binding_t only_active[] = {d[0], d[1], d[2],
        {.kind=SOLAR_OS_EXPANSION_BINDING_PARAMETER, .role="cd_active", .value=0}};
    assert(solar_os_sdmmc_attach("storage0", only_active, 4) == ESP_ERR_INVALID_ARG);
    solar_os_expansion_binding_t bad_level[] = {d[0], d[1], d[2], d[3],
        {.kind=SOLAR_OS_EXPANSION_BINDING_PARAMETER, .role="cd_active", .value=2}};
    assert(solar_os_sdmmc_attach("storage0", bad_level, 5) == ESP_ERR_INVALID_ARG);
    assert(!sdmmc.active);
    solar_os_expansion_binding_t twice[] = {d[0], d[1], d[2], d[3], d[3]};
    assert(solar_os_sdmmc_attach("storage0", twice, 5) == ESP_ERR_INVALID_ARG);

    /* Active-low switch (the PicoCalc): card seated at start, pulled for four
     * samples, then reinserted. Contact bounce in between changes nothing. */
    gpio_configs = tasks_created = tasks_deleted = 0;
    storage_mounts = storage_unmounts = 0;
    level_script = "0" "1" "0" "1" "1" "1" "1" "0" "1" "0" "0" "0" "0" "0";
    level_index = 0;
    assert(solar_os_sdmmc_attach("storage0", d, 4) == ESP_OK);
    assert(gpio_configs == 1 && gpio_pin_masks[0] == (1U << 6) && gpio_pullups[0] == GPIO_PULLUP_ENABLE);
    assert(tasks_created == 1 && sdmmc.cd_pin == 6 && sdmmc.cd_present_level == 0);
    task_function(task_argument);
    assert(tasks_deleted == 1);
    /* One removal and one insertion; the bounce at the start and between them
     * produced neither. */
    assert(storage_unmounts == 1);
    assert(storage_mounts == 1);
    sdmmc.detect_stop = false;
    assert(solar_os_sdmmc_detach("storage0") == ESP_OK && sdmmc.detect_task == NULL);

    /* Active-high switch: level 1 means a card is present. */
    storage_mounts = storage_unmounts = 0;
    solar_os_expansion_binding_t high[] = {d[0], d[1], d[2], d[3],
        {.kind=SOLAR_OS_EXPANSION_BINDING_PARAMETER, .role="cd_active", .value=1}};
    level_script = "0" "0" "0" "0" "1" "1" "1" "1";
    level_index = 0;
    assert(solar_os_sdmmc_attach("storage0", high, 5) == ESP_OK && sdmmc.cd_present_level == 1);
    task_function(task_argument);
    /* Started empty, a card arrives. */
    assert(storage_mounts == 1 && storage_unmounts == 0);
    sdmmc.detect_stop = false;
    assert(solar_os_sdmmc_detach("storage0") == ESP_OK);

    /* A card that will not mount at first is retried and then given up on. */
    storage_mounts = storage_unmounts = 0;
    storage_mount_error = ESP_ERR_TIMEOUT;
    level_script = "1" "1" "1" "1" "0" "0" "0" "0";
    level_index = 0;
    assert(solar_os_sdmmc_attach("storage0", d, 4) == ESP_OK);
    task_function(task_argument);
    assert(storage_mounts == SDMMC_DETECT_MOUNT_ATTEMPTS);
    storage_mount_error = ESP_OK;
    sdmmc.detect_stop = false;
    assert(solar_os_sdmmc_detach("storage0") == ESP_OK);

    /* Detach waits for the polling task, and refuses if it will not stop. */
    level_script = "0" "0";
    level_index = 0;
    assert(solar_os_sdmmc_attach("storage0", d, 4) == ESP_OK);
    wait_done_result = false;
    assert(solar_os_sdmmc_detach("storage0") == ESP_ERR_TIMEOUT && sdmmc.active);
    wait_done_result = true;
    assert(solar_os_sdmmc_detach("storage0") == ESP_OK && !sdmmc.active);

    /* The card still works if the polling task cannot start. */
    task_create_fails = true;
    assert(solar_os_sdmmc_attach("storage0", d, 4) == ESP_OK && sdmmc.cd_pin < 0);
    assert(solar_os_sdmmc_detach("storage0") == ESP_OK);
    task_create_fails = false;
    puts("SDMMC card-detect tests passed");
}
