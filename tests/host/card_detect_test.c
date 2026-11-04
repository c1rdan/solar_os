#include <assert.h>
#include <stdio.h>

#include "solar_os_card_detect.h"

static solar_os_card_detect_event_t feed(solar_os_card_detect_t *d, const char *levels)
{
    solar_os_card_detect_event_t last = SOLAR_OS_CARD_DETECT_NONE;
    for (; *levels != '\0'; levels++) {
        const solar_os_card_detect_event_t event =
            solar_os_card_detect_sample(d, *levels == '1');
        if (event != SOLAR_OS_CARD_DETECT_NONE) {
            last = event;
        }
    }
    return last;
}

int main(void)
{
    solar_os_card_detect_t d;

    /* Pin levels map through the polarity that means "card present". */
    assert(solar_os_card_detect_present_from_level(0, 0));
    assert(!solar_os_card_detect_present_from_level(1, 0));
    assert(solar_os_card_detect_present_from_level(1, 1));
    assert(!solar_os_card_detect_present_from_level(0, 1));
    assert(solar_os_card_detect_present_from_level(5, 1));

    /* A steady state never reports anything. */
    solar_os_card_detect_init(&d, false, 3U);
    assert(feed(&d, "0000000") == SOLAR_OS_CARD_DETECT_NONE);

    /* Insertion needs three consecutive samples and reports exactly once. */
    solar_os_card_detect_init(&d, false, 3U);
    assert(feed(&d, "11") == SOLAR_OS_CARD_DETECT_NONE);
    assert(solar_os_card_detect_sample(&d, true) == SOLAR_OS_CARD_DETECT_INSERTED);
    assert(d.present);
    assert(feed(&d, "1111") == SOLAR_OS_CARD_DETECT_NONE);

    /* Removal likewise. */
    assert(feed(&d, "00") == SOLAR_OS_CARD_DETECT_NONE);
    assert(solar_os_card_detect_sample(&d, false) == SOLAR_OS_CARD_DETECT_REMOVED);
    assert(!d.present);

    /* Contact bounce while inserting never completes a change. */
    solar_os_card_detect_init(&d, false, 3U);
    assert(feed(&d, "1101101101100") == SOLAR_OS_CARD_DETECT_NONE);
    assert(!d.present);

    /* ...but settling after the bounce does. */
    assert(feed(&d, "1101111") == SOLAR_OS_CARD_DETECT_INSERTED);

    /* Bounce while a card is held must not unmount it. */
    solar_os_card_detect_init(&d, true, 3U);
    assert(feed(&d, "1011010110111") == SOLAR_OS_CARD_DETECT_NONE);
    assert(d.present);

    /* A full cycle yields one event per transition. */
    solar_os_card_detect_init(&d, true, 2U);
    assert(feed(&d, "00") == SOLAR_OS_CARD_DETECT_REMOVED);
    assert(feed(&d, "11") == SOLAR_OS_CARD_DETECT_INSERTED);
    assert(feed(&d, "00") == SOLAR_OS_CARD_DETECT_REMOVED);

    /* Zero required samples is treated as one; NULL is harmless. */
    solar_os_card_detect_init(&d, false, 0U);
    assert(solar_os_card_detect_sample(&d, true) == SOLAR_OS_CARD_DETECT_INSERTED);
    solar_os_card_detect_init(NULL, false, 3U);
    assert(solar_os_card_detect_sample(NULL, true) == SOLAR_OS_CARD_DETECT_NONE);

    puts("card_detect_test: ok");
    return 0;
}
