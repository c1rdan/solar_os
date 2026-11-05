from pathlib import Path
import tempfile
import unittest

from tests.test_shutdown_policy import run_c

ROOT = Path(__file__).resolve().parents[1]

STUBS = {
    "driver/rmt_encoder.h": r'''
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#define RMT_ENCODER_FUNC_ATTR
#ifndef __containerof
#define __containerof(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))
#endif
typedef struct rmt_channel_t *rmt_channel_handle_t;
typedef enum {
    RMT_ENCODING_RESET = 0,
    RMT_ENCODING_COMPLETE = 1,
    RMT_ENCODING_MEM_FULL = 2,
} rmt_encode_state_t;
typedef struct rmt_encoder_t rmt_encoder_t;
typedef rmt_encoder_t *rmt_encoder_handle_t;
struct rmt_encoder_t {
    size_t (*encode)(rmt_encoder_t *, rmt_channel_handle_t, const void *, size_t,
                     rmt_encode_state_t *);
    esp_err_t (*reset)(rmt_encoder_t *);
    esp_err_t (*del)(rmt_encoder_t *);
};
typedef union {
    struct {
        uint16_t duration0 : 15;
        uint16_t level0 : 1;
        uint16_t duration1 : 15;
        uint16_t level1 : 1;
    };
    uint32_t val;
} rmt_symbol_word_t;
typedef struct {
    rmt_symbol_word_t bit0, bit1;
    struct { uint32_t msb_first : 1; } flags;
} rmt_bytes_encoder_config_t;
typedef struct { int unused; } rmt_copy_encoder_config_t;
void *rmt_alloc_encoder_mem(size_t size);
esp_err_t rmt_new_bytes_encoder(const rmt_bytes_encoder_config_t *config,
                                rmt_encoder_handle_t *encoder);
esp_err_t rmt_new_copy_encoder(const rmt_copy_encoder_config_t *config,
                               rmt_encoder_handle_t *encoder);
esp_err_t rmt_encoder_reset(rmt_encoder_handle_t encoder);
esp_err_t rmt_del_encoder(rmt_encoder_handle_t encoder);
''',
    "driver/rmt_tx.h": r'''
#pragma once
#include "driver/rmt_encoder.h"
#define RMT_CLK_SRC_DEFAULT 4
typedef struct {
    int gpio_num, clk_src;
    uint32_t resolution_hz;
    size_t mem_block_symbols, trans_queue_depth;
    struct { uint32_t init_level : 1; } flags;
} rmt_tx_channel_config_t;
typedef struct {
    int loop_count;
    struct { uint32_t eot_level : 1; } flags;
} rmt_transmit_config_t;
esp_err_t rmt_new_tx_channel(const rmt_tx_channel_config_t *config,
                             rmt_channel_handle_t *channel);
esp_err_t rmt_del_channel(rmt_channel_handle_t channel);
esp_err_t rmt_enable(rmt_channel_handle_t channel);
esp_err_t rmt_disable(rmt_channel_handle_t channel);
esp_err_t rmt_transmit(rmt_channel_handle_t channel, rmt_encoder_handle_t encoder,
                       const void *payload, size_t bytes,
                       const rmt_transmit_config_t *config);
esp_err_t rmt_tx_wait_all_done(rmt_channel_handle_t channel, int timeout_ms);
''',
    "driver/gpio.h": r'''
#pragma once
#include "esp_err.h"
typedef int gpio_num_t;
#define GPIO_IS_VALID_GPIO(pin) ((pin) >= 0 && (pin) < 49)
#define GPIO_IS_VALID_OUTPUT_GPIO(pin) GPIO_IS_VALID_GPIO(pin)
esp_err_t gpio_reset_pin(gpio_num_t pin);
esp_err_t gpio_sleep_sel_dis(gpio_num_t pin);
esp_err_t gpio_sleep_sel_en(gpio_num_t pin);
''',
    "esp_rom_sys.h": "#pragma once\n#include <stdint.h>\nvoid esp_rom_delay_us(uint32_t us);\n",
    "sdkconfig.h": "#pragma once\n",
}

SOURCE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "neopixel.h"

#define PIN 21

/* RMT model: an enabled channel holds a PM lock that blocks light sleep. */
static struct {
    bool created, enabled;
    int pm_locks, idle_level, frames, frames_while_disabled;
    int fail_enable, fail_transmit, fail_wait, busy_disables;
    const uint8_t *payload;
    size_t payload_bytes;
} rmt;
static rmt_tx_channel_config_t channel_config;
static rmt_bytes_encoder_config_t bytes_config;
static int live_encoders, delays;
/* GPIO model: the reset workaround isolates every pad in sleep by default. */
static bool output[49], pulled_up[49], sleep_switch[49];

const char *esp_err_to_name(esp_err_t err) { (void)err; return "error"; }
void esp_rom_delay_us(uint32_t us) { (void)us; delays++; }

static esp_err_t sub_encoder_reset(rmt_encoder_t *encoder) { (void)encoder; return ESP_OK; }
static esp_err_t sub_encoder_del(rmt_encoder_t *encoder) { free(encoder); live_encoders--; return ESP_OK; }
static esp_err_t new_sub_encoder(rmt_encoder_handle_t *encoder)
{
    rmt_encoder_t *sub = calloc(1, sizeof(*sub));
    assert(sub != NULL);
    sub->reset = sub_encoder_reset;
    sub->del = sub_encoder_del;
    live_encoders++;
    *encoder = sub;
    return ESP_OK;
}
void *rmt_alloc_encoder_mem(size_t size) { live_encoders++; return calloc(1, size); }
esp_err_t rmt_new_bytes_encoder(const rmt_bytes_encoder_config_t *config,
                                rmt_encoder_handle_t *encoder)
{
    bytes_config = *config;
    return new_sub_encoder(encoder);
}
esp_err_t rmt_new_copy_encoder(const rmt_copy_encoder_config_t *config,
                               rmt_encoder_handle_t *encoder)
{
    (void)config;
    return new_sub_encoder(encoder);
}
esp_err_t rmt_encoder_reset(rmt_encoder_handle_t encoder) { return encoder->reset(encoder); }
esp_err_t rmt_del_encoder(rmt_encoder_handle_t encoder)
{
    const bool top = encoder->encode != NULL;
    const esp_err_t ret = encoder->del(encoder);
    live_encoders -= top ? 1 : 0;
    return ret;
}

esp_err_t rmt_new_tx_channel(const rmt_tx_channel_config_t *config,
                             rmt_channel_handle_t *channel)
{
    assert(!rmt.created);
    channel_config = *config;
    rmt.created = true;
    rmt.idle_level = config->flags.init_level;
    output[config->gpio_num] = true;
    *channel = (rmt_channel_handle_t)&rmt;
    return ESP_OK;
}
esp_err_t rmt_del_channel(rmt_channel_handle_t channel)
{
    assert(channel == (rmt_channel_handle_t)&rmt && rmt.created);
    if (rmt.enabled) return ESP_ERR_INVALID_STATE;
    rmt.created = false;
    output[channel_config.gpio_num] = false;
    return ESP_OK;
}
esp_err_t rmt_enable(rmt_channel_handle_t channel)
{
    assert(channel == (rmt_channel_handle_t)&rmt && rmt.created);
    if (rmt.enabled) return ESP_ERR_INVALID_STATE;
    if (rmt.fail_enable) return ESP_FAIL;
    rmt.enabled = true;
    rmt.pm_locks++;
    return ESP_OK;
}
esp_err_t rmt_disable(rmt_channel_handle_t channel)
{
    assert(channel == (rmt_channel_handle_t)&rmt && rmt.created);
    if (!rmt.enabled) return ESP_ERR_INVALID_STATE;
    /* The done interrupt has not left its busy state yet. */
    if (rmt.busy_disables > 0) {
        rmt.busy_disables--;
        return ESP_ERR_INVALID_STATE;
    }
    rmt.enabled = false;
    rmt.pm_locks--;
    return ESP_OK;
}
esp_err_t rmt_transmit(rmt_channel_handle_t channel, rmt_encoder_handle_t encoder,
                       const void *payload, size_t bytes,
                       const rmt_transmit_config_t *config)
{
    assert(channel == (rmt_channel_handle_t)&rmt && encoder != NULL);
    assert(config->loop_count == 0 && config->flags.eot_level == 0);
    if (rmt.fail_transmit) return ESP_ERR_INVALID_STATE;
    rmt.idle_level = config->flags.eot_level;
    rmt.payload = payload;
    rmt.payload_bytes = bytes;
    rmt.frames++;
    rmt.frames_while_disabled += rmt.enabled ? 0 : 1;
    return ESP_OK;
}
esp_err_t rmt_tx_wait_all_done(rmt_channel_handle_t channel, int timeout_ms)
{
    assert(channel == (rmt_channel_handle_t)&rmt && timeout_ms > 0);
    return rmt.fail_wait ? ESP_ERR_TIMEOUT : ESP_OK;
}

esp_err_t gpio_reset_pin(gpio_num_t pin)
{
    output[pin] = false;
    pulled_up[pin] = true;
    return ESP_OK;
}
esp_err_t gpio_sleep_sel_dis(gpio_num_t pin) { sleep_switch[pin] = false; return ESP_OK; }
esp_err_t gpio_sleep_sel_en(gpio_num_t pin) { sleep_switch[pin] = true; return ESP_OK; }

static bool light_sleep_allowed(void) { return rmt.pm_locks == 0; }
static bool line_low_through_sleep(void)
{
    return output[PIN] && !sleep_switch[PIN] && rmt.idle_level == 0;
}
static bool idle(void) { return !rmt.enabled && light_sleep_allowed() && line_low_through_sleep(); }

int main(void)
{
    for (int pin = 0; pin < 49; pin++) sleep_switch[pin] = DEFAULT_SLEEP_SWITCH;
    const uint8_t pixel[3] = {0x10, 0x20, 0x30};
    const uint8_t strip_bytes[6] = {1, 2, 3, 4, 5, 6};
    neopixel_t strip;

    assert(neopixel_init(&strip, PIN) == ESP_OK);
    /* Timing and latch match the WS2812 protocol as before. */
    assert(channel_config.gpio_num == PIN && channel_config.resolution_hz == 10000000U);
    assert(channel_config.mem_block_symbols == 64 && channel_config.trans_queue_depth == 1);
    assert(channel_config.flags.init_level == 0);
    assert(bytes_config.bit0.level0 == 1 && bytes_config.bit0.duration0 == 3);
    assert(bytes_config.bit0.level1 == 0 && bytes_config.bit0.duration1 == 9);
    assert(bytes_config.bit1.level0 == 1 && bytes_config.bit1.duration0 == 9);
    assert(bytes_config.bit1.level1 == 0 && bytes_config.bit1.duration1 == 3);
    assert(bytes_config.flags.msb_first == 1);
    /* An attached but idle strip must not keep the chip out of light sleep. */
    assert(idle());

    for (int i = 0; i < 50; i++) {
        assert(neopixel_write(&strip, i % 2 ? pixel : strip_bytes, i % 2 ? 3 : 6) == ESP_OK);
        assert(rmt.payload == (i % 2 ? pixel : strip_bytes));
        assert(rmt.payload_bytes == (i % 2 ? 3U : 6U));
        assert(idle());
    }
    assert(rmt.frames == 50 && rmt.frames_while_disabled == 0);

    /* Rejected arguments do not touch the channel. */
    assert(neopixel_write(&strip, NULL, 3) == ESP_ERR_INVALID_ARG);
    assert(neopixel_write(&strip, pixel, 2) == ESP_ERR_INVALID_ARG);
    assert(rmt.frames == 50 && idle());

    /* Failures release the channel and its lock. */
    rmt.fail_enable = 1;
    assert(neopixel_write(&strip, pixel, 3) == ESP_FAIL);
    rmt.fail_enable = 0;
    assert(rmt.frames == 50 && idle());
    rmt.fail_transmit = 1;
    assert(neopixel_write(&strip, pixel, 3) == ESP_ERR_INVALID_STATE);
    rmt.fail_transmit = 0;
    assert(idle());
    rmt.fail_wait = 1;
    assert(neopixel_write(&strip, pixel, 3) == ESP_ERR_TIMEOUT);
    rmt.fail_wait = 0;
    assert(idle());
    assert(neopixel_write(&strip, pixel, 3) == ESP_OK && idle());

    /* A disable that races the done interrupt is retried. */
    rmt.busy_disables = 3;
    delays = 0;
    assert(neopixel_write(&strip, pixel, 3) == ESP_OK);
    assert(delays == 3 && idle());

    /* A channel left enabled is reused and released by the next frame. */
    rmt.busy_disables = 1000;
    assert(neopixel_write(&strip, pixel, 3) == ESP_ERR_INVALID_STATE);
    assert(rmt.enabled && !light_sleep_allowed());
    rmt.busy_disables = 0;
    const int frames = rmt.frames;
    assert(neopixel_write(&strip, pixel, 3) == ESP_OK);
    assert(rmt.frames == frames + 1 && idle());

    /* Detach also frees a channel that is still enabled, with the same
     * retry for a racing done interrupt. */
    rmt.busy_disables = 1000;
    assert(neopixel_write(&strip, pixel, 3) == ESP_ERR_INVALID_STATE);
    rmt.busy_disables = 3;
    neopixel_deinit(&strip);
    assert(!rmt.created && light_sleep_allowed() && live_encoders == 0);
    assert(strip.channel == NULL && strip.data_pin == -1);
    assert(!output[PIN] && pulled_up[PIN]);
    /* The pin gets back IDF's default sleep state. */
    assert(sleep_switch[PIN] == DEFAULT_SLEEP_SWITCH);
    assert(neopixel_write(&strip, pixel, 3) == ESP_ERR_INVALID_ARG);

    /* Reattach works the same way. */
    assert(neopixel_init(&strip, PIN) == ESP_OK && idle());
    assert(neopixel_write(&strip, pixel, 3) == ESP_OK && idle());
    neopixel_deinit(&strip);
    assert(!rmt.created && light_sleep_allowed() && live_encoders == 0);
    return 0;
}
'''


class NeopixelSleepTest(unittest.TestCase):
    def test_idle_strip_allows_light_sleep_and_holds_the_line_low(self):
        configs = (
            ("isolated sleep pads", ["-DCONFIG_ESP_SLEEP_GPIO_RESET_WORKAROUND=1",
                                     "-DDEFAULT_SLEEP_SWITCH=1"]),
            ("no sleep pad switching", ["-DDEFAULT_SLEEP_SWITCH=0"]),
        )
        with tempfile.TemporaryDirectory() as directory:
            stubs = Path(directory)
            for name, text in STUBS.items():
                (stubs / name).parent.mkdir(parents=True, exist_ok=True)
                (stubs / name).write_text(text)
            for name, flags in configs:
                with self.subTest(name):
                    # The encoder's fall-through comment is not in GCC's
                    # host -Wimplicit-fallthrough wording.
                    run_c(SOURCE, "-I", str(stubs), "-I", str(ROOT / "src/drivers"),
                          "-Wno-implicit-fallthrough", *flags,
                          str(ROOT / "src/drivers/neopixel.c"))


if __name__ == "__main__":
    unittest.main()
