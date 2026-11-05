#include "solar_os_picocalc_codec.h"

#include <stddef.h>
#include <string.h>

#include "solar_os_input.h"
#include "solar_os_keys.h"

/* Key codes from the PicoCalc keyboard controller firmware (keyboard.h). */
#define PICOCALC_KEY_BACKSPACE 0x08U
#define PICOCALC_KEY_TAB 0x09U
#define PICOCALC_KEY_ENTER 0x0aU
#define PICOCALC_KEY_F1 0x81U
#define PICOCALC_KEY_F9 0x89U
#define PICOCALC_KEY_F10 0x90U
#define PICOCALC_KEY_POWER 0x91U
#define PICOCALC_KEY_MOD_ALT 0xa1U
#define PICOCALC_KEY_MOD_SHL 0xa2U
#define PICOCALC_KEY_MOD_SHR 0xa3U
#define PICOCALC_KEY_MOD_SYM 0xa4U
#define PICOCALC_KEY_MOD_CTRL 0xa5U
#define PICOCALC_KEY_ESC 0xb1U
#define PICOCALC_KEY_LEFT 0xb4U
#define PICOCALC_KEY_UP 0xb5U
#define PICOCALC_KEY_DOWN 0xb6U
#define PICOCALC_KEY_RIGHT 0xb7U
#define PICOCALC_KEY_CAPS_LOCK 0xc1U
#define PICOCALC_KEY_BREAK 0xd0U
#define PICOCALC_KEY_INSERT 0xd1U
#define PICOCALC_KEY_HOME 0xd2U
#define PICOCALC_KEY_DEL 0xd4U
#define PICOCALC_KEY_END 0xd5U
#define PICOCALC_KEY_PAGE_UP 0xd6U
#define PICOCALC_KEY_PAGE_DOWN 0xd7U

/* Shifted character -> the unshifted character on the same physical key. */
static const char shifted_pairs[][2] = {
    {'~', '`'}, {'!', '1'}, {'@', '2'}, {'#', '3'}, {'$', '4'}, {'%', '5'},
    {'^', '6'}, {'&', '7'}, {'*', '8'}, {'(', '9'}, {')', '0'}, {'_', '-'},
    {'+', '='}, {'|', '\\'}, {'?', '/'}, {'>', '.'}, {'<', ','}, {':', ';'},
    {'"', '\''}, {'{', '['}, {'}', ']'},
};

static uint8_t modifier_bit(uint8_t code)
{
    switch (code) {
    case PICOCALC_KEY_MOD_CTRL: return SOLAR_OS_INPUT_MOD_LEFT_CTRL;
    case PICOCALC_KEY_MOD_ALT: return SOLAR_OS_INPUT_MOD_LEFT_ALT;
    case PICOCALC_KEY_MOD_SHL: return SOLAR_OS_INPUT_MOD_LEFT_SHIFT;
    case PICOCALC_KEY_MOD_SHR: return SOLAR_OS_INPUT_MOD_RIGHT_SHIFT;
    default: return 0U;
    }
}

static bool is_modifier(uint8_t code)
{
    return code == PICOCALC_KEY_MOD_ALT || code == PICOCALC_KEY_MOD_SHL ||
        code == PICOCALC_KEY_MOD_SHR || code == PICOCALC_KEY_MOD_SYM ||
        code == PICOCALC_KEY_MOD_CTRL;
}

/* Map a code to the identity of the physical key that produced it. */
static uint8_t physical_id(uint8_t code)
{
    if (code >= 'A' && code <= 'Z') {
        return (uint8_t)(code - 'A' + 'a');
    }
    for (size_t i = 0; i < sizeof(shifted_pairs) / sizeof(shifted_pairs[0]); i++) {
        if ((uint8_t)shifted_pairs[i][0] == code) {
            return (uint8_t)shifted_pairs[i][1];
        }
    }
    switch (code) {
    /* Shift+F1..F5 report F6..F10; F10 sits apart from F1..F9 in the table. */
    case PICOCALC_KEY_F1 + 5U:
    case PICOCALC_KEY_F1 + 6U:
    case PICOCALC_KEY_F1 + 7U:
    case PICOCALC_KEY_F1 + 8U:
        return (uint8_t)(code - 5U);
    case PICOCALC_KEY_F10: return (uint8_t)(PICOCALC_KEY_F1 + 4U);
    case PICOCALC_KEY_END: return PICOCALC_KEY_DEL;
    case PICOCALC_KEY_HOME: return PICOCALC_KEY_TAB;
    case PICOCALC_KEY_BREAK: return PICOCALC_KEY_ESC;
    case PICOCALC_KEY_INSERT: return PICOCALC_KEY_ENTER;
    case PICOCALC_KEY_PAGE_UP: return PICOCALC_KEY_UP;
    case PICOCALC_KEY_PAGE_DOWN: return PICOCALC_KEY_DOWN;
    default: return code;
    }
}

/* Translate a controller key code to a SolarOS logical key; 0 if none. */
static uint8_t logical_key(uint8_t code, uint8_t held_modifiers)
{
    if (code >= PICOCALC_KEY_F1 && code <= PICOCALC_KEY_F9) {
        return (uint8_t)(SOLAR_OS_KEY_F1 + (code - PICOCALC_KEY_F1));
    }
    if (code >= 0x20U && code <= 0x7eU) {
        /* With Alt held the controller leaves letters in their uppercase
         * table form. Hand SolarOS the lowercase letter unless Shift is held. */
        if (code >= 'A' && code <= 'Z' &&
            (held_modifiers & SOLAR_OS_INPUT_MOD_ALT) != 0U &&
            (held_modifiers & SOLAR_OS_INPUT_MOD_SHIFT) == 0U) {
            return (uint8_t)(code - 'A' + 'a');
        }
        return code;
    }
    switch (code) {
    case PICOCALC_KEY_BACKSPACE: return '\b';
    case PICOCALC_KEY_TAB: return '\t';
    case PICOCALC_KEY_ENTER: return SOLAR_OS_KEY_ENTER;
    case PICOCALC_KEY_ESC: return SOLAR_OS_KEY_ESCAPE;
    case PICOCALC_KEY_UP: return SOLAR_OS_KEY_UP;
    case PICOCALC_KEY_DOWN: return SOLAR_OS_KEY_DOWN;
    case PICOCALC_KEY_LEFT: return SOLAR_OS_KEY_LEFT;
    case PICOCALC_KEY_RIGHT: return SOLAR_OS_KEY_RIGHT;
    case PICOCALC_KEY_PAGE_UP: return SOLAR_OS_KEY_PAGE_UP;
    case PICOCALC_KEY_PAGE_DOWN: return SOLAR_OS_KEY_PAGE_DOWN;
    case PICOCALC_KEY_HOME: return SOLAR_OS_KEY_HOME;
    case PICOCALC_KEY_END: return SOLAR_OS_KEY_END;
    case PICOCALC_KEY_DEL: return SOLAR_OS_KEY_DELETE;
    /* Break (Shift+Esc) is the PicoCalc's interrupt key. */
    case PICOCALC_KEY_BREAK: return SOLAR_OS_KEY_APP_EXIT;
    case PICOCALC_KEY_F10: return SOLAR_OS_KEY_F10;
    default: return 0U;
    }
}

static solar_os_picocalc_down_key_t *find_down(
    solar_os_picocalc_keyboard_t *keyboard,
    uint8_t id,
    size_t *index)
{
    for (size_t i = 0; i < keyboard->down_count; i++) {
        if (keyboard->down[i].id == id) {
            if (index != NULL) {
                *index = i;
            }
            return &keyboard->down[i];
        }
    }
    return NULL;
}

static void remove_down(solar_os_picocalc_keyboard_t *keyboard, size_t index)
{
    keyboard->down_count--;
    if (index < keyboard->down_count) {
        memmove(&keyboard->down[index],
                &keyboard->down[index + 1U],
                (keyboard->down_count - index) * sizeof(keyboard->down[0]));
    }
}

void solar_os_picocalc_keyboard_reset(solar_os_picocalc_keyboard_t *keyboard)
{
    if (keyboard != NULL) {
        memset(keyboard, 0, sizeof(*keyboard));
    }
}

bool solar_os_picocalc_keyboard_fifo_empty(uint8_t state, uint8_t code)
{
    return state == SOLAR_OS_PICOCALC_STATE_IDLE && code == 0U;
}

bool solar_os_picocalc_keyboard_fifo_valid(uint8_t state, uint8_t code)
{
    return state <= SOLAR_OS_PICOCALC_STATE_RELEASED &&
        (state != SOLAR_OS_PICOCALC_STATE_IDLE || code == 0U);
}

solar_os_picocalc_key_result_t solar_os_picocalc_keyboard_decode(
    solar_os_picocalc_keyboard_t *keyboard,
    uint8_t state,
    uint8_t code,
    solar_os_picocalc_key_transition_t *transition)
{
    if (keyboard == NULL || transition == NULL ||
        state == SOLAR_OS_PICOCALC_STATE_IDLE || state > SOLAR_OS_PICOCALC_STATE_RELEASED ||
        code == 0U) {
        return SOLAR_OS_PICOCALC_KEY_NONE;
    }

    if (is_modifier(code)) {
        const uint8_t bit = modifier_bit(code);
        if (state == SOLAR_OS_PICOCALC_STATE_RELEASED) {
            keyboard->held_modifiers &= (uint8_t)~bit;
        } else {
            keyboard->held_modifiers |= bit;
        }
        return SOLAR_OS_PICOCALC_KEY_NONE;
    }
    if (code == PICOCALC_KEY_CAPS_LOCK) {
        /* The controller applies Caps Lock to the letters it reports. */
        return SOLAR_OS_PICOCALC_KEY_NONE;
    }
    if (code == PICOCALC_KEY_POWER) {
        return state == SOLAR_OS_PICOCALC_STATE_PRESSED ?
            SOLAR_OS_PICOCALC_KEY_POWER : SOLAR_OS_PICOCALC_KEY_NONE;
    }

    const uint8_t id = physical_id(code);
    size_t index = 0U;
    solar_os_picocalc_down_key_t *down = find_down(keyboard, id, &index);

    if (state == SOLAR_OS_PICOCALC_STATE_RELEASED) {
        if (down == NULL) {
            return SOLAR_OS_PICOCALC_KEY_NONE;
        }
        *transition = (solar_os_picocalc_key_transition_t) {
            .physical_key = id,
            .key = down->key,
            .modifiers = down->modifiers,
            .pressed = false,
        };
        remove_down(keyboard, index);
        return SOLAR_OS_PICOCALC_KEY_TRANSITION;
    }

    /* PRESSED or HOLD. A key that is already down is a repeat of the
     * controller's own timer; SolarOS generates its own repeats. */
    if (down != NULL) {
        return SOLAR_OS_PICOCALC_KEY_NONE;
    }
    if (state == SOLAR_OS_PICOCALC_STATE_HOLD) {
        /* A HOLD with no recorded press: the press was lost or predates the
         * driver. Ignore it rather than invent a press. */
        return SOLAR_OS_PICOCALC_KEY_NONE;
    }

    const uint8_t key = logical_key(code, keyboard->held_modifiers);
    if (key == 0U) {
        return SOLAR_OS_PICOCALC_KEY_UNSUPPORTED;
    }
    if (keyboard->down_count >= SOLAR_OS_PICOCALC_KEYS_DOWN_MAX) {
        return SOLAR_OS_PICOCALC_KEY_NONE;
    }

    const uint8_t modifiers = keyboard->held_modifiers &
        (uint8_t)(SOLAR_OS_INPUT_MOD_CTRL | SOLAR_OS_INPUT_MOD_ALT);
    keyboard->down[keyboard->down_count++] = (solar_os_picocalc_down_key_t) {
        .id = id,
        .key = key,
        .modifiers = modifiers,
    };
    *transition = (solar_os_picocalc_key_transition_t) {
        .physical_key = id,
        .key = key,
        .modifiers = modifiers,
        .pressed = true,
    };
    return SOLAR_OS_PICOCALC_KEY_TRANSITION;
}

bool solar_os_picocalc_keyboard_pop_down(
    solar_os_picocalc_keyboard_t *keyboard,
    solar_os_picocalc_key_transition_t *transition)
{
    if (keyboard == NULL || transition == NULL || keyboard->down_count == 0U) {
        return false;
    }
    const solar_os_picocalc_down_key_t last = keyboard->down[--keyboard->down_count];
    *transition = (solar_os_picocalc_key_transition_t) {
        .physical_key = last.id,
        .key = last.key,
        .modifiers = last.modifiers,
        .pressed = false,
    };
    return true;
}

#define LCD_REG_MIN 16U
#define LCD_REG_MAX 240U
#define KBD_REG_STEP 32U
#define KBD_REG_MAX 224U

uint8_t solar_os_picocalc_lcd_register_from_percent(uint8_t percent)
{
    if (percent > 100U) {
        percent = 100U;
    }
    return (uint8_t)(LCD_REG_MIN +
                     ((uint32_t)percent * (LCD_REG_MAX - LCD_REG_MIN) + 50U) / 100U);
}

uint8_t solar_os_picocalc_lcd_percent_from_register(uint8_t value)
{
    if (value <= LCD_REG_MIN) {
        return 0U;
    }
    if (value >= LCD_REG_MAX) {
        return 100U;
    }
    return (uint8_t)(((uint32_t)(value - LCD_REG_MIN) * 100U +
                      (LCD_REG_MAX - LCD_REG_MIN) / 2U) /
                     (LCD_REG_MAX - LCD_REG_MIN));
}

uint8_t solar_os_picocalc_kbd_register_from_percent(uint8_t percent)
{
    if (percent == 0U) {
        return 0U;
    }
    if (percent > 100U) {
        percent = 100U;
    }
    /* Round to a whole step so the value is exact, and keep at least the
     * first step: anything below it switches the backlight off. */
    uint32_t steps = ((uint32_t)percent * (KBD_REG_MAX / KBD_REG_STEP) + 50U) / 100U;
    if (steps == 0U) {
        steps = 1U;
    }
    return (uint8_t)(steps * KBD_REG_STEP);
}

bool solar_os_picocalc_decode_battery(const uint8_t data[2],
                                      solar_os_picocalc_battery_t *battery)
{
    if (data == NULL || battery == NULL || data[0] != SOLAR_OS_PICOCALC_REG_BATTERY) {
        return false;
    }
    const uint8_t value = data[1];
    battery->charging = (value & 0x80U) != 0U;
    battery->percent = (uint8_t)(value & 0x7fU);
    if (battery->percent > 100U) {
        battery->percent = 100U;
    }
    battery->valid = value != 0U;
    return true;
}

uint16_t solar_os_picocalc_estimate_millivolts(uint8_t percent,
                                               uint16_t empty_mv,
                                               uint16_t full_mv)
{
    if (full_mv <= empty_mv) {
        return empty_mv;
    }
    if (percent > 100U) {
        percent = 100U;
    }
    return (uint16_t)(empty_mv +
                      ((uint32_t)percent * (uint32_t)(full_mv - empty_mv) + 50U) / 100U);
}
