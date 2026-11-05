#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "solar_os_board_display.h"

static unsigned panel_power_save_calls;
static uint8_t panel_power_save_last;

/* The board display layer only touches u8g2 through these. */
void u8x8_SetPowerSave(u8x8_t *u8x8, uint8_t is_enable)
{
    (void)u8x8;
    panel_power_save_calls++;
    panel_power_save_last = is_enable;
}

static unsigned provider_calls;
static uint8_t provider_last;
static void *provider_user_seen;
static esp_err_t provider_result;

static esp_err_t provider_set(void *user, uint8_t percent)
{
    provider_calls++;
    provider_last = percent;
    provider_user_seen = user;
    return provider_result;
}

static unsigned ops_calls;
static uint8_t ops_last;

static bool ops_supported(const solar_os_board_display_t *display)
{
    (void)display;
    return true;
}

static esp_err_t ops_get(const solar_os_board_display_t *display, uint8_t *percent)
{
    (void)display;
    *percent = 33U;
    return ESP_OK;
}

static esp_err_t ops_set(solar_os_board_display_t *display, uint8_t percent)
{
    (void)display;
    ops_calls++;
    ops_last = percent;
    return ESP_OK;
}

static esp_err_t failing_power_save(solar_os_board_display_t *display, bool enabled)
{
    (void)display;
    (void)enabled;
    return ESP_FAIL;
}

static const solar_os_board_display_ops_t pin_backlight_ops = {
    .brightness_supported = ops_supported,
    .get_brightness = ops_get,
    .set_brightness = ops_set,
};
/* A panel with no backlight of its own, like the PicoCalc's. */
static const solar_os_board_display_ops_t no_backlight_ops = {0};
static const solar_os_board_display_ops_t failing_power_save_ops = {
    .set_power_save = failing_power_save,
};

static void reset_counters(void)
{
    provider_calls = ops_calls = panel_power_save_calls = 0U;
    provider_last = ops_last = panel_power_save_last = 0xffU;
    provider_user_seen = NULL;
    provider_result = ESP_OK;
}

int main(void)
{
    int owner_a, owner_b;
    uint8_t percent = 0U;

    /* A display with no backlight and no controller bound: unsupported, but the
     * request is still remembered. */
    solar_os_board_display_t panel = {.ops = &no_backlight_ops};
    reset_counters();
    assert(!solar_os_board_display_brightness_supported(&panel));
    assert(solar_os_board_display_set_brightness(&panel, 40U) == ESP_ERR_NOT_SUPPORTED);
    assert(solar_os_board_display_get_brightness(&panel, &percent) == ESP_ERR_NOT_SUPPORTED);
    assert(provider_calls == 0U);

    /* Binding applies the level that was requested before it existed. */
    assert(solar_os_board_display_set_brightness_provider(provider_set, &owner_a) == ESP_OK);
    assert(provider_calls == 1U && provider_last == 40U && provider_user_seen == &owner_a);
    assert(solar_os_board_display_brightness_supported(&panel));
    assert(solar_os_board_display_get_brightness(&panel, &percent) == ESP_OK && percent == 40U);

    /* Later requests go to the provider, and are what get reports. */
    assert(solar_os_board_display_set_brightness(&panel, 70U) == ESP_OK);
    assert(provider_calls == 2U && provider_last == 70U);
    assert(solar_os_board_display_get_brightness(&panel, &percent) == ESP_OK && percent == 70U);

    /* Out-of-range requests are rejected and do not disturb the remembered one. */
    assert(solar_os_board_display_set_brightness(&panel, 101U) == ESP_ERR_INVALID_ARG);
    assert(provider_calls == 2U);
    assert(solar_os_board_display_get_brightness(&panel, &percent) == ESP_OK && percent == 70U);

    /* Only one provider may be bound, and a missing callback is invalid. */
    assert(solar_os_board_display_set_brightness_provider(provider_set, &owner_b) ==
           ESP_ERR_INVALID_STATE);
    assert(solar_os_board_display_set_brightness_provider(NULL, &owner_b) ==
           ESP_ERR_INVALID_ARG);
    assert(provider_calls == 2U);

    /* A failing controller reports the error but the level stays remembered. */
    provider_result = ESP_FAIL;
    assert(solar_os_board_display_set_brightness(&panel, 55U) == ESP_FAIL);
    assert(solar_os_board_display_get_brightness(&panel, &percent) == ESP_OK && percent == 55U);
    provider_result = ESP_OK;

    /* Only the owner can unbind. */
    solar_os_board_display_clear_brightness_provider(&owner_b);
    assert(solar_os_board_display_brightness_supported(&panel));
    solar_os_board_display_clear_brightness_provider(&owner_a);
    assert(!solar_os_board_display_brightness_supported(&panel));
    assert(solar_os_board_display_set_brightness(&panel, 10U) == ESP_ERR_NOT_SUPPORTED);

    /* A controller that attaches again starts from the latest request. */
    reset_counters();
    assert(solar_os_board_display_set_brightness_provider(provider_set, &owner_b) == ESP_OK);
    assert(provider_calls == 1U && provider_last == 10U && provider_user_seen == &owner_b);
    solar_os_board_display_clear_brightness_provider(&owner_b);

    /* A panel with its own backlight is untouched until a provider takes over. */
    solar_os_board_display_t pin_panel = {.ops = &pin_backlight_ops};
    reset_counters();
    assert(solar_os_board_display_brightness_supported(&pin_panel));
    assert(solar_os_board_display_get_brightness(&pin_panel, &percent) == ESP_OK && percent == 33U);
    assert(solar_os_board_display_set_brightness(&pin_panel, 80U) == ESP_OK);
    assert(ops_calls == 1U && ops_last == 80U && provider_calls == 0U);

    assert(solar_os_board_display_set_brightness_provider(provider_set, &owner_a) == ESP_OK);
    assert(provider_calls == 1U && provider_last == 80U);
    assert(solar_os_board_display_set_brightness(&pin_panel, 20U) == ESP_OK);
    /* The provider replaces the pin backlight rather than driving both. */
    assert(provider_calls == 2U && provider_last == 20U && ops_calls == 1U);
    solar_os_board_display_clear_brightness_provider(&owner_a);
    assert(solar_os_board_display_set_brightness(&pin_panel, 90U) == ESP_OK);
    assert(ops_calls == 2U && ops_last == 90U);

    assert(solar_os_board_display_set_brightness(NULL, 50U) == ESP_ERR_NOT_SUPPORTED);
    assert(solar_os_board_display_get_brightness(&pin_panel, NULL) == ESP_ERR_INVALID_ARG);

    /* Power save holds a provider backlight at its lowest level. */
    u8g2_t u8g2;
    memset(&u8g2, 0, sizeof(u8g2));
    panel.u8g2 = &u8g2;
    assert(solar_os_board_display_set_brightness(&panel, 60U) == ESP_ERR_NOT_SUPPORTED);
    reset_counters();
    assert(solar_os_board_display_set_brightness_provider(provider_set, &owner_a) == ESP_OK);
    assert(provider_calls == 1U && provider_last == 60U);
    assert(solar_os_board_display_set_power_save(&panel, true) == ESP_OK);
    assert(panel_power_save_calls == 1U && panel_power_save_last == 1U);
    assert(provider_calls == 2U && provider_last == 0U && provider_user_seen == &owner_a);
    assert(solar_os_board_display_get_brightness(&panel, &percent) == ESP_OK && percent == 60U);

    /* A request while asleep is remembered, not applied. */
    assert(solar_os_board_display_set_brightness(&panel, 30U) == ESP_OK);
    assert(provider_calls == 2U && provider_last == 0U);
    assert(solar_os_board_display_get_brightness(&panel, &percent) == ESP_OK && percent == 30U);

    /* Wake restores the latest request. */
    assert(solar_os_board_display_set_power_save(&panel, false) == ESP_OK);
    assert(panel_power_save_calls == 2U && panel_power_save_last == 0U);
    assert(provider_calls == 3U && provider_last == 30U);
    assert(solar_os_board_display_set_brightness(&panel, 45U) == ESP_OK);
    assert(provider_calls == 4U && provider_last == 45U);

    /* A provider that binds while asleep starts at its lowest level. */
    solar_os_board_display_clear_brightness_provider(&owner_a);
    assert(solar_os_board_display_set_power_save(&panel, true) == ESP_OK);
    reset_counters();
    assert(solar_os_board_display_set_brightness_provider(provider_set, &owner_b) == ESP_OK);
    assert(provider_calls == 1U && provider_last == 0U && provider_user_seen == &owner_b);
    assert(solar_os_board_display_set_power_save(&panel, false) == ESP_OK);
    assert(provider_calls == 2U && provider_last == 45U);

    /* A failing backlight does not fail the panel transition. */
    provider_result = ESP_FAIL;
    assert(solar_os_board_display_set_power_save(&panel, true) == ESP_OK);
    assert(provider_calls == 3U && provider_last == 0U);
    provider_result = ESP_OK;
    assert(solar_os_board_display_set_brightness(&panel, 50U) == ESP_OK);
    assert(provider_calls == 3U);
    assert(solar_os_board_display_set_power_save(&panel, false) == ESP_OK);
    assert(provider_calls == 4U && provider_last == 50U);

    /* A panel that fails to sleep leaves the backlight alone and stays awake. */
    solar_os_board_display_t failing_panel = {.ops = &failing_power_save_ops, .u8g2 = &u8g2};
    reset_counters();
    assert(solar_os_board_display_set_power_save(&failing_panel, true) == ESP_FAIL);
    assert(provider_calls == 0U && panel_power_save_calls == 0U);
    assert(solar_os_board_display_set_brightness(&panel, 55U) == ESP_OK);
    assert(provider_calls == 1U && provider_last == 55U);
    solar_os_board_display_clear_brightness_provider(&owner_b);

    /* A pin backlight is still left to its driver, asleep or not. */
    pin_panel.u8g2 = &u8g2;
    reset_counters();
    assert(solar_os_board_display_set_power_save(&pin_panel, true) == ESP_OK);
    assert(panel_power_save_calls == 1U && ops_calls == 0U && provider_calls == 0U);
    assert(solar_os_board_display_set_brightness(&pin_panel, 70U) == ESP_OK);
    assert(ops_calls == 1U && ops_last == 70U);
    assert(solar_os_board_display_set_power_save(&pin_panel, false) == ESP_OK);
    assert(ops_calls == 1U && provider_calls == 0U);

    puts("board_display_brightness_test: ok");
    return 0;
}
