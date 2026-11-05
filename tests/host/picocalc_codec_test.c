#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "solar_os_input.h"
#include "solar_os_keys.h"
#include "solar_os_picocalc_codec.h"

#define PRESSED SOLAR_OS_PICOCALC_STATE_PRESSED
#define HOLD SOLAR_OS_PICOCALC_STATE_HOLD
#define RELEASED SOLAR_OS_PICOCALC_STATE_RELEASED

#define MOD_ALT 0xa1U
#define MOD_SHL 0xa2U
#define MOD_SHR 0xa3U
#define MOD_SYM 0xa4U
#define MOD_CTRL 0xa5U

static solar_os_picocalc_key_result_t decode(
    solar_os_picocalc_keyboard_t *kb,
    uint8_t state,
    uint8_t code,
    solar_os_picocalc_key_transition_t *t)
{
    return solar_os_picocalc_keyboard_decode(kb, state, code, t);
}

static void expect_press(solar_os_picocalc_keyboard_t *kb,
                         uint8_t code,
                         uint8_t physical,
                         uint8_t key,
                         uint8_t modifiers)
{
    solar_os_picocalc_key_transition_t t = {0};
    assert(decode(kb, PRESSED, code, &t) == SOLAR_OS_PICOCALC_KEY_TRANSITION);
    assert(t.pressed);
    assert(t.physical_key == physical);
    assert(t.key == key);
    assert(t.modifiers == modifiers);
}

static void expect_release(solar_os_picocalc_keyboard_t *kb,
                           uint8_t code,
                           uint8_t physical,
                           uint8_t key,
                           uint8_t modifiers)
{
    solar_os_picocalc_key_transition_t t = {0};
    assert(decode(kb, RELEASED, code, &t) == SOLAR_OS_PICOCALC_KEY_TRANSITION);
    assert(!t.pressed);
    assert(t.physical_key == physical);
    assert(t.key == key);
    assert(t.modifiers == modifiers);
}

static void expect_none(solar_os_picocalc_keyboard_t *kb, uint8_t state, uint8_t code)
{
    solar_os_picocalc_key_transition_t t = {0};
    assert(decode(kb, state, code, &t) == SOLAR_OS_PICOCALC_KEY_NONE);
}

static void tap(solar_os_picocalc_keyboard_t *kb, uint8_t code, uint8_t key)
{
    expect_press(kb, code, code, key, 0U);
    expect_release(kb, code, code, key, 0U);
    assert(kb->down_count == 0U);
}

static void test_fifo_empty(void)
{
    solar_os_picocalc_keyboard_t kb;
    solar_os_picocalc_keyboard_reset(&kb);
    assert(solar_os_picocalc_keyboard_fifo_empty(0U, 0U));
    assert(!solar_os_picocalc_keyboard_fifo_empty(1U, 'a'));
    expect_none(&kb, 0U, 0U);
    expect_none(&kb, 4U, 'a');
    expect_none(&kb, PRESSED, 0U);
}

static void test_fifo_valid(void)
{
    assert(solar_os_picocalc_keyboard_fifo_valid(0U, 0U));
    assert(solar_os_picocalc_keyboard_fifo_valid(PRESSED, 'a'));
    assert(solar_os_picocalc_keyboard_fifo_valid(HOLD, 0x82U));
    assert(solar_os_picocalc_keyboard_fifo_valid(RELEASED, 'a'));

    /* Replies to other registers: the version {0, 0x16}, the battery, both
     * backlights and the power-off register echo a register number or carry a
     * value with state 0. */
    assert(!solar_os_picocalc_keyboard_fifo_valid(0U, 0x16U));
    assert(!solar_os_picocalc_keyboard_fifo_valid(4U, 0U));
    assert(!solar_os_picocalc_keyboard_fifo_valid(0x05U, 0x50U));
    assert(!solar_os_picocalc_keyboard_fifo_valid(0x0aU, 0x40U));
    assert(!solar_os_picocalc_keyboard_fifo_valid(0x0bU, 0x55U));
    assert(!solar_os_picocalc_keyboard_fifo_valid(0x0eU, 1U));
    assert(!solar_os_picocalc_keyboard_fifo_valid(0xffU, 0xffU));
}

static void test_plain_key_and_repeats(void)
{
    solar_os_picocalc_keyboard_t kb;
    solar_os_picocalc_keyboard_reset(&kb);

    expect_press(&kb, 'a', 'a', 'a', 0U);
    /* The controller's own repeats must not become extra presses. */
    expect_none(&kb, HOLD, 'a');
    expect_none(&kb, PRESSED, 'a');
    expect_none(&kb, PRESSED, 'a');
    assert(kb.down_count == 1U);
    expect_release(&kb, 'a', 'a', 'a', 0U);
    assert(kb.down_count == 0U);

    /* A release with no recorded press is ignored. */
    expect_none(&kb, RELEASED, 'a');
    /* So is a HOLD that has no recorded press. */
    expect_none(&kb, HOLD, 'b');
}

static void test_release_after_case_change(void)
{
    solar_os_picocalc_keyboard_t kb;
    solar_os_picocalc_keyboard_reset(&kb);

    /* Pressed as "a", Shift goes down while it is held, released as "A". */
    expect_press(&kb, 'a', 'a', 'a', 0U);
    expect_none(&kb, PRESSED, MOD_SHL);
    expect_none(&kb, PRESSED, 'A');
    expect_release(&kb, 'A', 'a', 'a', 0U);
    expect_none(&kb, RELEASED, MOD_SHL);
    assert(kb.down_count == 0U);

    /* Shift+1 pressed as "!" and released as "1", and the reverse. */
    expect_none(&kb, PRESSED, MOD_SHR);
    expect_press(&kb, '!', '1', '!', 0U);
    expect_none(&kb, RELEASED, MOD_SHR);
    expect_release(&kb, '1', '1', '!', 0U);

    expect_press(&kb, '1', '1', '1', 0U);
    expect_none(&kb, PRESSED, MOD_SHL);
    expect_release(&kb, '!', '1', '1', 0U);
    expect_none(&kb, RELEASED, MOD_SHL);
    assert(kb.down_count == 0U);
}

static void test_symbol_identities(void)
{
    static const char pairs[][2] = {
        {'~', '`'}, {'!', '1'}, {'@', '2'}, {'#', '3'}, {'$', '4'}, {'%', '5'},
        {'^', '6'}, {'&', '7'}, {'*', '8'}, {'(', '9'}, {')', '0'}, {'_', '-'},
        {'+', '='}, {'|', '\\'}, {'?', '/'}, {'>', '.'}, {'<', ','}, {':', ';'},
        {'"', '\''}, {'{', '['}, {'}', ']'},
    };
    for (unsigned i = 0; i < sizeof(pairs) / sizeof(pairs[0]); i++) {
        solar_os_picocalc_keyboard_t kb;
        solar_os_picocalc_keyboard_reset(&kb);
        expect_press(&kb, (uint8_t)pairs[i][0], (uint8_t)pairs[i][1],
                     (uint8_t)pairs[i][0], 0U);
        expect_release(&kb, (uint8_t)pairs[i][1], (uint8_t)pairs[i][1],
                       (uint8_t)pairs[i][0], 0U);
    }
    for (char c = 'A'; c <= 'Z'; c++) {
        solar_os_picocalc_keyboard_t kb;
        solar_os_picocalc_keyboard_reset(&kb);
        expect_press(&kb, (uint8_t)c, (uint8_t)(c - 'A' + 'a'), (uint8_t)c, 0U);
        expect_release(&kb, (uint8_t)(c - 'A' + 'a'), (uint8_t)(c - 'A' + 'a'),
                       (uint8_t)c, 0U);
    }
}

static void test_modifiers(void)
{
    solar_os_picocalc_keyboard_t kb;
    solar_os_picocalc_keyboard_reset(&kb);

    /* Modifier events are bookkeeping, never published. */
    expect_none(&kb, PRESSED, MOD_CTRL);
    assert(kb.held_modifiers == SOLAR_OS_INPUT_MOD_LEFT_CTRL);
    expect_none(&kb, HOLD, MOD_CTRL);
    /* Ctrl is published for the consumer to turn into a control character. */
    expect_press(&kb, 'c', 'c', 'c', SOLAR_OS_INPUT_MOD_LEFT_CTRL);
    /* Dropping Ctrl before the key still releases with the press's modifiers. */
    expect_none(&kb, RELEASED, MOD_CTRL);
    assert(kb.held_modifiers == 0U);
    expect_release(&kb, 'c', 'c', 'c', SOLAR_OS_INPUT_MOD_LEFT_CTRL);

    /* Shift is already applied by the controller and is not re-published. */
    expect_none(&kb, PRESSED, MOD_SHL);
    expect_press(&kb, 'A', 'a', 'A', 0U);
    expect_release(&kb, 'A', 'a', 'A', 0U);
    expect_none(&kb, RELEASED, MOD_SHL);

    /* Shift+Up arrives as Page Up and must not become Shift+Page Up. */
    expect_none(&kb, PRESSED, MOD_SHR);
    expect_press(&kb, 0xd6U, 0xb5U, SOLAR_OS_KEY_PAGE_UP, 0U);
    expect_release(&kb, 0xb5U, 0xb5U, SOLAR_OS_KEY_PAGE_UP, 0U);
    expect_none(&kb, RELEASED, MOD_SHR);

    /* Sym has no physical key; it must not set anything. */
    expect_none(&kb, PRESSED, MOD_SYM);
    assert(kb.held_modifiers == 0U);

    /* Ctrl and Alt together. */
    expect_none(&kb, PRESSED, MOD_CTRL);
    expect_none(&kb, PRESSED, MOD_ALT);
    expect_press(&kb, 'x', 'x', 'x',
                 SOLAR_OS_INPUT_MOD_LEFT_CTRL | SOLAR_OS_INPUT_MOD_LEFT_ALT);
}

static void test_alt_lowercases_letters(void)
{
    solar_os_picocalc_keyboard_t kb;
    solar_os_picocalc_keyboard_reset(&kb);

    /* With Alt held the controller leaves letters uppercase. */
    expect_none(&kb, PRESSED, MOD_ALT);
    expect_press(&kb, 'F', 'f', 'f', SOLAR_OS_INPUT_MOD_LEFT_ALT);
    expect_release(&kb, 'F', 'f', 'f', SOLAR_OS_INPUT_MOD_LEFT_ALT);

    /* Alt+Shift keeps the capital. */
    expect_none(&kb, PRESSED, MOD_SHL);
    expect_press(&kb, 'F', 'f', 'F', SOLAR_OS_INPUT_MOD_LEFT_ALT);
}

static void test_special_keys(void)
{
    solar_os_picocalc_keyboard_t kb;
    solar_os_picocalc_keyboard_reset(&kb);

    tap(&kb, 0x0aU, SOLAR_OS_KEY_ENTER);
    tap(&kb, 0x08U, '\b');
    tap(&kb, 0x09U, '\t');
    tap(&kb, ' ', ' ');
    tap(&kb, 0xb1U, SOLAR_OS_KEY_ESCAPE);
    tap(&kb, 0xb5U, SOLAR_OS_KEY_UP);
    tap(&kb, 0xb6U, SOLAR_OS_KEY_DOWN);
    tap(&kb, 0xb4U, SOLAR_OS_KEY_LEFT);
    tap(&kb, 0xb7U, SOLAR_OS_KEY_RIGHT);
    tap(&kb, 0xd4U, SOLAR_OS_KEY_DELETE);

    /* Shifted forms share the physical key's identity. */
    expect_press(&kb, 0xd5U, 0xd4U, SOLAR_OS_KEY_END, 0U);
    expect_release(&kb, 0xd5U, 0xd4U, SOLAR_OS_KEY_END, 0U);
    expect_press(&kb, 0xd2U, 0x09U, SOLAR_OS_KEY_HOME, 0U);
    expect_release(&kb, 0xd2U, 0x09U, SOLAR_OS_KEY_HOME, 0U);
    expect_press(&kb, 0xd7U, 0xb6U, SOLAR_OS_KEY_PAGE_DOWN, 0U);
    expect_release(&kb, 0xd7U, 0xb6U, SOLAR_OS_KEY_PAGE_DOWN, 0U);
    expect_press(&kb, 0xd0U, 0xb1U, SOLAR_OS_KEY_APP_EXIT, 0U);
    expect_release(&kb, 0xd0U, 0xb1U, SOLAR_OS_KEY_APP_EXIT, 0U);
    assert(kb.down_count == 0U);
}

static void test_function_keys(void)
{
    solar_os_picocalc_keyboard_t kb;
    solar_os_picocalc_keyboard_reset(&kb);

    /* The unshifted row is F1..F5. */
    for (uint8_t i = 0; i < 5U; i++) {
        tap(&kb, (uint8_t)(0x81U + i), (uint8_t)(SOLAR_OS_KEY_F1 + i));
    }
    /* Shift+F1..F5 report F6..F10 (F10 is 0x90, apart from F1..F9) and share
     * the unshifted key's physical identity. */
    for (uint8_t i = 0; i < 4U; i++) {
        const uint8_t shifted = (uint8_t)(0x86U + i);
        const uint8_t physical = (uint8_t)(0x81U + i);
        const uint8_t key = (uint8_t)(SOLAR_OS_KEY_F6 + i);
        expect_press(&kb, shifted, physical, key, 0U);
        expect_release(&kb, shifted, physical, key, 0U);
    }
    expect_press(&kb, 0x90U, 0x85U, SOLAR_OS_KEY_F10, 0U);
    expect_release(&kb, 0x90U, 0x85U, SOLAR_OS_KEY_F10, 0U);
    assert(kb.down_count == 0U);

    /* Pressed as Shift+F1 and released after Shift went up: still F1's key. */
    expect_press(&kb, 0x86U, 0x81U, SOLAR_OS_KEY_F6, 0U);
    expect_release(&kb, 0x81U, 0x81U, SOLAR_OS_KEY_F6, 0U);
    /* And pressed as F5, released as Shift+F5 (F10). */
    expect_press(&kb, 0x85U, 0x85U, SOLAR_OS_KEY_F5, 0U);
    expect_release(&kb, 0x90U, 0x85U, SOLAR_OS_KEY_F5, 0U);
    assert(kb.down_count == 0U);

    /* A function key sends HOLD once held; it is not a repeat press. */
    expect_press(&kb, 0x82U, 0x82U, SOLAR_OS_KEY_F2, 0U);
    expect_none(&kb, HOLD, 0x82U);
    expect_none(&kb, HOLD, 0x82U);
    expect_release(&kb, 0x82U, 0x82U, SOLAR_OS_KEY_F2, 0U);
}

static void test_unsupported_and_power(void)
{
    solar_os_picocalc_keyboard_t kb;
    solar_os_picocalc_keyboard_reset(&kb);
    solar_os_picocalc_key_transition_t t;

    /* Insert and the joystick buttons have no SolarOS logical key. */
    assert(decode(&kb, PRESSED, 0xd1U, &t) == SOLAR_OS_PICOCALC_KEY_UNSUPPORTED);
    assert(decode(&kb, PRESSED, 0x06U, &t) == SOLAR_OS_PICOCALC_KEY_UNSUPPORTED);
    assert(decode(&kb, PRESSED, 0x11U, &t) == SOLAR_OS_PICOCALC_KEY_UNSUPPORTED);
    assert(kb.down_count == 0U);

    /* Caps Lock is applied by the controller. */
    expect_none(&kb, PRESSED, 0xc1U);
    expect_none(&kb, RELEASED, 0xc1U);

    /* The power key reports once per press and is never tracked as held. */
    assert(decode(&kb, PRESSED, 0x91U, &t) == SOLAR_OS_PICOCALC_KEY_POWER);
    expect_none(&kb, HOLD, 0x91U);
    expect_none(&kb, RELEASED, 0x91U);
    assert(kb.down_count == 0U);
}

static void test_overflow_and_pop_down(void)
{
    solar_os_picocalc_keyboard_t kb;
    solar_os_picocalc_keyboard_reset(&kb);
    solar_os_picocalc_key_transition_t t;

    /* 26 distinct keys against a 24-entry table: the last two are dropped
     * without disturbing the ones already held. */
    uint8_t pressed = 0U;
    for (uint8_t c = 'a'; c <= 'z'; c++) {
        const solar_os_picocalc_key_result_t result = decode(&kb, PRESSED, c, &t);
        if (result == SOLAR_OS_PICOCALC_KEY_TRANSITION) {
            pressed++;
        } else {
            assert(result == SOLAR_OS_PICOCALC_KEY_NONE);
        }
    }
    assert(pressed == SOLAR_OS_PICOCALC_KEYS_DOWN_MAX);
    assert(kb.down_count == SOLAR_OS_PICOCALC_KEYS_DOWN_MAX);
    expect_release(&kb, 'a', 'a', 'a', 0U);
    expect_none(&kb, RELEASED, 'z');
    expect_press(&kb, 'z', 'z', 'z', 0U);
    pressed = (uint8_t)kb.down_count;

    unsigned released = 0U;
    while (solar_os_picocalc_keyboard_pop_down(&kb, &t)) {
        assert(!t.pressed);
        released++;
    }
    assert(released == pressed);
    assert(kb.down_count == 0U);
    assert(!solar_os_picocalc_keyboard_pop_down(&kb, &t));
}

static void test_null_arguments(void)
{
    solar_os_picocalc_keyboard_t kb;
    solar_os_picocalc_key_transition_t t;
    solar_os_picocalc_keyboard_reset(NULL);
    solar_os_picocalc_keyboard_reset(&kb);
    assert(decode(NULL, PRESSED, 'a', &t) == SOLAR_OS_PICOCALC_KEY_NONE);
    assert(decode(&kb, PRESSED, 'a', NULL) == SOLAR_OS_PICOCALC_KEY_NONE);
    assert(!solar_os_picocalc_keyboard_pop_down(NULL, &t));
}

static void test_lcd_backlight_register(void)
{
    /* The controller clamps to 16..240, so 0 percent must not ask for less. */
    assert(solar_os_picocalc_lcd_register_from_percent(0U) == 16U);
    assert(solar_os_picocalc_lcd_register_from_percent(100U) == 240U);
    assert(solar_os_picocalc_lcd_register_from_percent(200U) == 240U);

    uint8_t previous = 0U;
    for (unsigned percent = 0U; percent <= 100U; percent++) {
        const uint8_t value = solar_os_picocalc_lcd_register_from_percent((uint8_t)percent);
        assert(value >= 16U && value <= 240U);
        assert(value >= previous);
        previous = value;
        /* Mapping back must land on the requested level. */
        assert(solar_os_picocalc_lcd_percent_from_register(value) == percent);
    }

    assert(solar_os_picocalc_lcd_percent_from_register(0U) == 0U);
    assert(solar_os_picocalc_lcd_percent_from_register(16U) == 0U);
    assert(solar_os_picocalc_lcd_percent_from_register(240U) == 100U);
    assert(solar_os_picocalc_lcd_percent_from_register(255U) == 100U);
}

static void test_keyboard_backlight_register(void)
{
    assert(solar_os_picocalc_kbd_register_from_percent(0U) == 0U);
    /* Any non-zero request keeps the backlight on. */
    assert(solar_os_picocalc_kbd_register_from_percent(1U) == 32U);
    assert(solar_os_picocalc_kbd_register_from_percent(100U) == 224U);
    assert(solar_os_picocalc_kbd_register_from_percent(250U) == 224U);

    uint8_t previous = 0U;
    for (unsigned percent = 0U; percent <= 100U; percent++) {
        const uint8_t value = solar_os_picocalc_kbd_register_from_percent((uint8_t)percent);
        /* Whole steps only, and never into the range the controller wraps to
         * off (above 240). */
        assert(value % 32U == 0U);
        assert(value <= 224U);
        assert(value >= previous);
        if (percent > 0U) {
            assert(value >= 32U);
        }
        previous = value;
    }
}

static void test_battery_register(void)
{
    solar_os_picocalc_battery_t battery = {0};
    const uint8_t none[2] = {0x0bU, 0x00U};
    assert(solar_os_picocalc_decode_battery(none, &battery));
    assert(!battery.valid && !battery.charging && battery.percent == 0U);

    const uint8_t discharging[2] = {0x0bU, 0x55U};
    assert(solar_os_picocalc_decode_battery(discharging, &battery));
    assert(battery.valid && !battery.charging && battery.percent == 85U);

    const uint8_t charging[2] = {0x0bU, (uint8_t)(0x80U | 85U)};
    assert(solar_os_picocalc_decode_battery(charging, &battery));
    assert(battery.valid && battery.charging && battery.percent == 85U);

    /* Charging at an empty gauge is still a real reading. */
    const uint8_t empty_charging[2] = {0x0bU, 0x80U};
    assert(solar_os_picocalc_decode_battery(empty_charging, &battery));
    assert(battery.valid && battery.charging && battery.percent == 0U);

    const uint8_t garbage[2] = {0x0bU, 0x7fU};
    assert(solar_os_picocalc_decode_battery(garbage, &battery));
    assert(battery.valid && battery.percent == 100U);

    assert(!solar_os_picocalc_decode_battery(NULL, &battery));
    assert(!solar_os_picocalc_decode_battery(discharging, NULL));
}

static void test_battery_wrong_register(void)
{
    /* A reply to another register must not replace the last reading: a key
     * entry {1, 'a'} would otherwise read as 97 percent. */
    static const uint8_t replies[][2] = {
        {0x01U, 'a'}, {0x03U, 'a'}, {0x00U, 0x16U}, {0x00U, 0x00U},
        {0x05U, 0x50U}, {0x0aU, 0x40U}, {0x0eU, 0x01U}, {0x8bU, 0x55U},
    };
    for (unsigned i = 0; i < sizeof(replies) / sizeof(replies[0]); i++) {
        solar_os_picocalc_battery_t battery = {.valid = true, .percent = 42U, .charging = true};
        assert(!solar_os_picocalc_decode_battery(replies[i], &battery));
        assert(battery.valid && battery.percent == 42U && battery.charging);
    }
}

static void test_voltage_estimate(void)
{
    assert(solar_os_picocalc_estimate_millivolts(0U, 3000U, 4120U) == 3000U);
    assert(solar_os_picocalc_estimate_millivolts(100U, 3000U, 4120U) == 4120U);
    assert(solar_os_picocalc_estimate_millivolts(50U, 3000U, 4120U) == 3560U);
    /* Out-of-range input clamps; a degenerate range returns its floor. */
    assert(solar_os_picocalc_estimate_millivolts(250U, 3000U, 4120U) == 4120U);
    assert(solar_os_picocalc_estimate_millivolts(50U, 4000U, 4000U) == 4000U);
    assert(solar_os_picocalc_estimate_millivolts(50U, 4200U, 3000U) == 4200U);

    uint16_t previous = 0U;
    for (unsigned percent = 0U; percent <= 100U; percent++) {
        const uint16_t mv = solar_os_picocalc_estimate_millivolts((uint8_t)percent, 3000U, 4120U);
        assert(mv >= 3000U && mv <= 4120U);
        assert(mv >= previous);
        previous = mv;
    }
}

int main(void)
{
    test_fifo_empty();
    test_fifo_valid();
    test_plain_key_and_repeats();
    test_release_after_case_change();
    test_symbol_identities();
    test_modifiers();
    test_alt_lowercases_letters();
    test_special_keys();
    test_function_keys();
    test_unsupported_and_power();
    test_overflow_and_pop_down();
    test_null_arguments();
    test_lcd_backlight_register();
    test_keyboard_backlight_register();
    test_battery_register();
    test_battery_wrong_register();
    test_voltage_estimate();
    puts("picocalc_codec_test: ok");
    return 0;
}
