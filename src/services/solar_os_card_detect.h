#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Debounce for a card-detect switch. A raw level is only accepted as a change
 * of card state once it has been seen for a number of consecutive samples, so
 * contact bounce while a card is inserted or removed does not mount and
 * unmount the volume repeatedly. */
typedef struct {
    bool present;
    bool candidate;
    uint8_t stable_samples;
    uint8_t required_samples;
} solar_os_card_detect_t;

typedef enum {
    SOLAR_OS_CARD_DETECT_NONE,
    SOLAR_OS_CARD_DETECT_INSERTED,
    SOLAR_OS_CARD_DETECT_REMOVED,
} solar_os_card_detect_event_t;

/* Start in the given state; a change needs required_samples (at least 1)
 * consecutive samples to take effect. */
void solar_os_card_detect_init(solar_os_card_detect_t *detect,
                               bool present,
                               uint8_t required_samples);

/* Convert a pin level to "card present" given the level that means present. */
bool solar_os_card_detect_present_from_level(int level, int present_level);

/* Feed one raw sample; reports an event when the debounced state changes. */
solar_os_card_detect_event_t solar_os_card_detect_sample(
    solar_os_card_detect_t *detect, bool present);
