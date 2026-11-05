#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Protocol helpers for the Clockwork PicoCalc keyboard and power controller:
 * the key FIFO decoder, the backlight register encodings and the battery
 * register. Pure logic with no hardware access, so it is host-testable.
 *
 * Key FIFO decoder.
 *
 * The controller is an STM32 that scans the matrix and has already resolved
 * Shift, Caps Lock and the symbol layer, so most entries are finished
 * characters. Register 0x09 returns two bytes per entry: the key state, then
 * the key code. Its state machine has three quirks this decoder absorbs:
 *
 *  - It reports the character as it stands when the event is generated. A key
 *    pressed as "a" and released after Shift went down is released as "A", and
 *    Shift+1 releases as "!" or "1" depending on timing. Keys are therefore
 *    tracked by a canonical physical identity (the unshifted code), never by
 *    the character, or a release would be lost and the key would stick.
 *  - Printable keys are re-sent as PRESSED about every 100 ms while held and
 *    other keys are re-sent as HOLD. SolarOS owns auto-repeat, so repeats of a
 *    key that is already down are dropped.
 *  - Modifiers arrive as ordinary key events. Shift is not published: the
 *    controller has already applied it, and re-applying it would turn its
 *    Shift+Up (Page Up) into Shift+Page Up.
 */

#define SOLAR_OS_PICOCALC_STATE_IDLE 0U
#define SOLAR_OS_PICOCALC_STATE_PRESSED 1U
#define SOLAR_OS_PICOCALC_STATE_HOLD 2U
#define SOLAR_OS_PICOCALC_STATE_RELEASED 3U

#define SOLAR_OS_PICOCALC_KEYS_DOWN_MAX 24U

typedef struct {
    uint8_t id;
    uint8_t key;
    uint8_t modifiers;
} solar_os_picocalc_down_key_t;

typedef struct {
    /* HID modifier bits for the modifier keys currently held. */
    uint8_t held_modifiers;
    uint8_t down_count;
    solar_os_picocalc_down_key_t down[SOLAR_OS_PICOCALC_KEYS_DOWN_MAX];
} solar_os_picocalc_keyboard_t;

typedef struct {
    /* Canonical physical key identity. Never zero. */
    uint8_t physical_key;
    /* SolarOS logical key byte: an ASCII character or a SOLAR_OS_KEY_* code. */
    uint8_t key;
    /* HID modifier bits to publish with the key (Ctrl and Alt only). */
    uint8_t modifiers;
    bool pressed;
} solar_os_picocalc_key_transition_t;

typedef enum {
    /* Empty FIFO entry, a modifier or lock change, or a repeat to ignore. */
    SOLAR_OS_PICOCALC_KEY_NONE,
    /* Publish the transition. */
    SOLAR_OS_PICOCALC_KEY_TRANSITION,
    /* A real key that SolarOS has no logical code for. */
    SOLAR_OS_PICOCALC_KEY_UNSUPPORTED,
    /* The mainboard power key was pressed. */
    SOLAR_OS_PICOCALC_KEY_POWER,
} solar_os_picocalc_key_result_t;

void solar_os_picocalc_keyboard_reset(solar_os_picocalc_keyboard_t *keyboard);

/* A FIFO read of {0, 0} means the controller's queue is empty. */
bool solar_os_picocalc_keyboard_fifo_empty(uint8_t state, uint8_t code);

/* False for a FIFO read the controller never sends: a state above 3, or state
 * 0 with a key code. Such a reply answers some other register. */
bool solar_os_picocalc_keyboard_fifo_valid(uint8_t state, uint8_t code);

solar_os_picocalc_key_result_t solar_os_picocalc_keyboard_decode(
    solar_os_picocalc_keyboard_t *keyboard,
    uint8_t state,
    uint8_t code,
    solar_os_picocalc_key_transition_t *transition);

/* Release every tracked key and return its release transitions one at a time.
 * Used when the controller stops answering so no key is left held. */
bool solar_os_picocalc_keyboard_pop_down(
    solar_os_picocalc_keyboard_t *keyboard,
    solar_os_picocalc_key_transition_t *transition);

/* True while any key or modifier is tracked as down. */
bool solar_os_picocalc_keyboard_any_down(const solar_os_picocalc_keyboard_t *keyboard);

/* Key FIFO polling. The controller has no interrupt line, so the host polls,
 * and each poll keeps the CPU out of light sleep for the whole transfer.
 *
 *  - Fast while a key or modifier is down, so a held key's release is seen
 *    within one of the input service's repeats (15 per second by default),
 *    and for the linger time after any FIFO entry, so typing stays fast.
 *  - Idle otherwise. The controller scans about every 20 ms, so the first
 *    key after a pause is read within about 20 ms + idle + one exchange.
 *  - Slower while the screen is off, where the key that wakes it waits for
 *    the panel to restart anyway.
 *
 * The controller queues 31 entries, far more than one wait can fill, so a
 * slow poll only delays keys. It resets its I2C peripheral after 2.5 s
 * without traffic, so no interval may come near that. */
#define SOLAR_OS_PICOCALC_POLL_FAST_MS 10U
#define SOLAR_OS_PICOCALC_POLL_IDLE_MS 40U
#define SOLAR_OS_PICOCALC_POLL_SCREEN_OFF_MS 100U
#define SOLAR_OS_PICOCALC_POLL_LINGER_MS 1000U
#define SOLAR_OS_PICOCALC_POLL_MAX_MS 1000U

typedef struct {
    uint16_t fast_ms;
    uint16_t idle_ms;
    uint16_t screen_off_ms;
    uint16_t linger_ms;
} solar_os_picocalc_poll_t;

typedef enum {
    SOLAR_OS_PICOCALC_POLL_FAST,
    SOLAR_OS_PICOCALC_POLL_IDLE,
    SOLAR_OS_PICOCALC_POLL_SCREEN_OFF,
} solar_os_picocalc_poll_mode_t;

void solar_os_picocalc_poll_defaults(solar_os_picocalc_poll_t *poll);

/* since_entry_ms is the time since the last non-empty FIFO read. */
solar_os_picocalc_poll_mode_t solar_os_picocalc_poll_mode(const solar_os_picocalc_poll_t *poll,
                                                          bool keys_down,
                                                          uint32_t since_entry_ms,
                                                          bool screen_off);

/* The wait before the next poll in a mode, kept within 1..POLL_MAX_MS. */
uint32_t solar_os_picocalc_poll_wait_ms(const solar_os_picocalc_poll_t *poll,
                                        solar_os_picocalc_poll_mode_t mode);

/* Controller registers (write: register | 0x80, then the value; read: write
 * the register, wait, read two bytes). */
#define SOLAR_OS_PICOCALC_REG_VERSION 0x01U
#define SOLAR_OS_PICOCALC_REG_KEY_COUNT 0x04U
#define SOLAR_OS_PICOCALC_REG_LCD_BACKLIGHT 0x05U
#define SOLAR_OS_PICOCALC_REG_RESET 0x08U
#define SOLAR_OS_PICOCALC_REG_FIFO 0x09U
#define SOLAR_OS_PICOCALC_REG_KBD_BACKLIGHT 0x0aU
#define SOLAR_OS_PICOCALC_REG_BATTERY 0x0bU
#define SOLAR_OS_PICOCALC_REG_POWER_OFF 0x0eU
#define SOLAR_OS_PICOCALC_REG_WRITE 0x80U

/* The controller quantises both backlight registers itself: the LCD to steps
 * of 16 within 16..240 (so the panel is never fully off), the keyboard to
 * steps of 32 within 32..224 where anything below 32 is off and anything
 * above 240 wraps to off. These map 0..100 percent onto the usable range. */
uint8_t solar_os_picocalc_lcd_register_from_percent(uint8_t percent);
uint8_t solar_os_picocalc_lcd_percent_from_register(uint8_t value);
uint8_t solar_os_picocalc_kbd_register_from_percent(uint8_t percent);

typedef struct {
    /* False when the controller reports 0: no battery, or no reading yet. */
    bool valid;
    uint8_t percent;
    bool charging;
} solar_os_picocalc_battery_t;

/* Decode the two bytes read from the battery register: {register, value} with
 * the percentage in bits 0..6 and the charging flag in bit 7. Returns false and
 * leaves *battery unchanged when the first byte is not the battery register:
 * the reply answers some other register. */
bool solar_os_picocalc_decode_battery(const uint8_t data[2],
                                      solar_os_picocalc_battery_t *battery);

/* The controller reports no voltage. Estimate one linearly from the
 * percentage over the battery service's configured empty..full range, so the
 * voltage-based consumers (trend, time left, the low-voltage guard) see a
 * value that agrees with the percentage instead of zero. */
uint16_t solar_os_picocalc_estimate_millivolts(uint8_t percent,
                                               uint16_t empty_mv,
                                               uint16_t full_mv);
