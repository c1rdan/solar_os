#include "solar_os_card_detect.h"

#include <stddef.h>

void solar_os_card_detect_init(solar_os_card_detect_t *detect,
                               bool present,
                               uint8_t required_samples)
{
    if (detect == NULL) {
        return;
    }
    detect->present = present;
    detect->candidate = present;
    detect->stable_samples = 0U;
    detect->required_samples = required_samples == 0U ? 1U : required_samples;
}

bool solar_os_card_detect_present_from_level(int level, int present_level)
{
    return (level != 0) == (present_level != 0);
}

solar_os_card_detect_event_t solar_os_card_detect_sample(
    solar_os_card_detect_t *detect, bool present)
{
    if (detect == NULL) {
        return SOLAR_OS_CARD_DETECT_NONE;
    }
    if (present == detect->present) {
        /* Agrees with the accepted state: any pending change was a glitch. */
        detect->candidate = detect->present;
        detect->stable_samples = 0U;
        return SOLAR_OS_CARD_DETECT_NONE;
    }
    if (present != detect->candidate) {
        detect->candidate = present;
        detect->stable_samples = 0U;
    }
    if (detect->stable_samples < UINT8_MAX) {
        detect->stable_samples++;
    }
    if (detect->stable_samples < detect->required_samples) {
        return SOLAR_OS_CARD_DETECT_NONE;
    }
    detect->present = present;
    detect->stable_samples = 0U;
    return present ? SOLAR_OS_CARD_DETECT_INSERTED : SOLAR_OS_CARD_DETECT_REMOVED;
}
