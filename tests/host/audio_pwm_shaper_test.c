#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Constants, the shaper typedef, audio_pwm_ramp_q4 and audio_pwm_shaper_step
 * from src/drivers/audio_pwm.c. */
#include "audio_pwm_shaper.inc"

_Static_assert(AUDIO_PWM_DUTY_MID_Q4 == 2048U, "midpoint in Q4 units");
_Static_assert(AUDIO_PWM_RAMP_SAMPLES == 640U, "40 ms at 16 kHz");
_Static_assert(AUDIO_PWM_USE_FRAC_DUTY == 1, "fractional duty is the default");

typedef int BaseType_t;
#define pdFALSE 0
#define pdTRUE 1
typedef void *gptimer_handle_t;
typedef struct { int unused; } gptimer_alarm_event_data_t;
typedef int ledc_channel_t;
#define AUDIO_PWM_CHANNEL 7
#define AUDIO_PWM_CHANNEL_RIGHT 6

/* Only the fields the sample interrupt uses. */
typedef struct {
    bool stereo;
    void *samples;
    audio_pwm_shaper_t shaper;
} audio_pwm_state_t;

static audio_pwm_state_t state;
static uint8_t queue[4096];
static unsigned queue_head, queue_tail, reads;
static uint16_t out[2];
static unsigned writes[2];

static size_t xStreamBufferReceiveFromISR(void *stream, void *data, size_t want, BaseType_t *woken)
{
    assert(stream == queue && want == (state.stereo ? 2U : 1U));
    ++reads;
    size_t got = 0;
    while (got < want && queue_head != queue_tail) {
        ((uint8_t *)data)[got++] = queue[queue_head++ % sizeof(queue)];
    }
    if (got != 0U) *woken = pdTRUE;
    return got;
}

static void audio_pwm_set_duty_q4(ledc_channel_t channel, uint16_t q4)
{
    assert(channel == AUDIO_PWM_CHANNEL || channel == AUDIO_PWM_CHANNEL_RIGHT);
    const unsigned c = channel == AUDIO_PWM_CHANNEL ? 0U : 1U;
    assert(q4 <= 255U << AUDIO_PWM_DUTY_FRAC_BITS);
    out[c] = q4;
    ++writes[c];
}

/* audio_pwm_on_sample from src/drivers/audio_pwm.c. */
#include "audio_pwm_isr.inc"

static unsigned step_bound(unsigned span, unsigned steps)
{
    return (3U * span + 2U * steps - 1U) / (2U * steps) + 1U;
}

static unsigned diff(uint16_t a, uint16_t b)
{
    return a > b ? (unsigned)(a - b) : (unsigned)(b - a);
}

static void start(bool stereo)
{
    memset(&state, 0, sizeof(state));
    state.stereo = stereo;
    state.samples = queue;
    state.shaper = (audio_pwm_shaper_t){.phase = AUDIO_PWM_PHASE_RAMP_UP};
    queue_head = queue_tail = reads = 0;
    out[0] = out[1] = 0;
    writes[0] = writes[1] = 0;
}

static void push(uint8_t left, uint8_t right)
{
    queue[queue_tail++ % sizeof(queue)] = left;
    if (state.stereo) queue[queue_tail++ % sizeof(queue)] = right;
}

static unsigned queued(void)
{
    return queue_tail - queue_head;
}

static bool tick(void)
{
    return audio_pwm_on_sample(NULL, NULL, &state);
}

/* The right channel is only written in stereo. */
static void check_right_written(unsigned expected)
{
    assert(writes[1] == (state.stereo ? expected : 0U));
}

static void ramp_helper(void)
{
    static const uint16_t targets[] = {0U, AUDIO_PWM_DUTY_MID_Q4, 4095U};
    static const unsigned lengths[] = {AUDIO_PWM_RAMP_SAMPLES, 320U, 1280U, 3200U};
    for (unsigned l = 0; l < sizeof(lengths) / sizeof(lengths[0]); l++) {
        const unsigned steps = lengths[l];
        /* Every start level at the real length, a sample of them otherwise. */
        const unsigned stride = steps == AUDIO_PWM_RAMP_SAMPLES ? 1U : 7U;
        for (unsigned from = 0; from <= 4095U; from += stride) {
            for (unsigned t = 0; t < 3U; t++) {
                const uint16_t to = targets[t];
                const unsigned bound = step_bound(diff((uint16_t)from, to), steps);
                uint16_t prev = audio_pwm_ramp_q4((uint16_t)from, to, 0U, steps);
                assert(prev == from);
                for (unsigned n = 1; n <= steps + 2U; n++) {
                    const uint16_t level = audio_pwm_ramp_q4((uint16_t)from, to, n, steps);
                    assert(to >= from ? level >= prev : level <= prev);
                    assert(diff(level, prev) <= bound);
                    if (n >= steps) assert(level == to);
                    prev = level;
                }
            }
        }
    }
}

/* Runs the ramp-up and checks it starts near 0 V, ends exactly at the
 * midpoint and never reads the queue. */
static void ramp_up(void)
{
    const unsigned bound = step_bound(AUDIO_PWM_DUTY_MID_Q4, AUDIO_PWM_RAMP_SAMPLES);
    const unsigned before = queued();
    uint16_t prev = 0;
    for (unsigned n = 1; n <= AUDIO_PWM_RAMP_SAMPLES; n++) {
        assert(state.shaper.phase == AUDIO_PWM_PHASE_RAMP_UP);
        assert(!tick());
        if (n == 1U) assert(out[0] < (1U << AUDIO_PWM_DUTY_FRAC_BITS));
        assert(out[0] >= prev && diff(out[0], prev) <= bound);
        if (state.stereo) assert(out[1] == out[0]);
        prev = out[0];
    }
    assert(out[0] == AUDIO_PWM_DUTY_MID_Q4);
    assert(state.shaper.phase == AUDIO_PWM_PHASE_PLAY);
    assert(state.shaper.ticks == AUDIO_PWM_RAMP_SAMPLES);
    assert(reads == 0U && queued() == before);
    assert(writes[0] == AUDIO_PWM_RAMP_SAMPLES);
    check_right_written(AUDIO_PWM_RAMP_SAMPLES);
}

/* Ticks until DONE, checking a smooth fall from each channel's own level. */
static unsigned ramp_down(uint16_t from_left, uint16_t from_right)
{
    const uint16_t from[2] = {from_left, state.stereo ? from_right : from_left};
    uint16_t prev[2] = {from[0], from[1]};
    unsigned n = 0;
    while (state.shaper.phase != AUDIO_PWM_PHASE_DONE) {
        const unsigned reads_before = reads;
        (void)tick();
        ++n;
        assert(n <= AUDIO_PWM_RAMP_SAMPLES);
        /* The first ramp-down tick is the one that found the queue empty. */
        assert(reads == reads_before + (n == 1U ? 1U : 0U));
        for (unsigned c = 0; c < (state.stereo ? 2U : 1U); c++) {
            assert(out[c] <= prev[c]);
            assert(diff(prev[c], out[c]) <= step_bound(from[c], AUDIO_PWM_RAMP_SAMPLES));
            prev[c] = out[c];
        }
    }
    assert(out[0] == 0U && (!state.stereo || out[1] == 0U));
    return n;
}

static void done_is_sticky(void)
{
    const unsigned ticks = state.shaper.ticks, left = writes[0], right = writes[1];
    const unsigned reads_before = reads;
    push(99, 99);
    for (unsigned i = 0; i < 100; i++) assert(!tick());
    assert(state.shaper.phase == AUDIO_PWM_PHASE_DONE);
    assert(state.shaper.ticks == ticks && writes[0] == left && writes[1] == right);
    assert(reads == reads_before && queued() == (state.stereo ? 2U : 1U));
}

static void play_underrun_and_drain(bool stereo)
{
    start(stereo);
    push(200, 60);
    ramp_up();

    /* PLAY outputs the sample in Q4. */
    assert(tick());
    assert(reads == 1U && queued() == 0U);
    assert(out[0] == 200U << AUDIO_PWM_DUTY_FRAC_BITS);
    if (stereo) assert(out[1] == 60U << AUDIO_PWM_DUTY_FRAC_BITS);
    /* Mono feeds its one sample to both levels. */
    else assert(state.shaper.level[1] == state.shaper.level[0]);
    check_right_written(AUDIO_PWM_RAMP_SAMPLES + 1U);

    /* An underrun holds the last level instead of jumping to the midpoint. */
    for (unsigned i = 0; i < 1000; i++) {
        assert(!tick());
        assert(state.shaper.phase == AUDIO_PWM_PHASE_PLAY);
        assert(out[0] == 200U << AUDIO_PWM_DUTY_FRAC_BITS);
        if (stereo) assert(out[1] == 60U << AUDIO_PWM_DUTY_FRAC_BITS);
    }

    /* Stop with frames queued: they play out first, then each channel falls
     * from its own level and DONE comes exactly one ramp after the drain. */
    for (unsigned i = 0; i < 50; i++) push((uint8_t)(100 + i), (uint8_t)(10 + i));
    push(200, 60);
    state.shaper.stop = true;
    for (unsigned i = 0; i < 50; i++) {
        assert(tick());
        assert(state.shaper.phase == AUDIO_PWM_PHASE_PLAY);
        assert(out[0] == (100U + i) << AUDIO_PWM_DUTY_FRAC_BITS);
        if (stereo) assert(out[1] == (10U + i) << AUDIO_PWM_DUTY_FRAC_BITS);
    }
    assert(tick());
    assert(out[0] == 200U << AUDIO_PWM_DUTY_FRAC_BITS);
    assert(queued() == 0U);
    assert(ramp_down(200U << AUDIO_PWM_DUTY_FRAC_BITS, 60U << AUDIO_PWM_DUTY_FRAC_BITS) ==
           AUDIO_PWM_RAMP_SAMPLES);
    done_is_sticky();
}

/* A stop during the ramp-up lets it finish, then falls straight back from
 * the midpoint without a jump at either boundary. */
static void stop_during_ramp_up(bool stereo)
{
    start(stereo);
    const unsigned bound = step_bound(AUDIO_PWM_DUTY_MID_Q4, AUDIO_PWM_RAMP_SAMPLES);
    uint16_t prev = 0;
    unsigned n = 0;
    while (state.shaper.phase != AUDIO_PWM_PHASE_DONE) {
        if (n == 100U) state.shaper.stop = true;
        (void)tick();
        ++n;
        assert(diff(out[0], prev) <= bound);
        if (stereo) assert(out[1] == out[0]);
        if (n <= AUDIO_PWM_RAMP_SAMPLES) assert(out[0] >= prev);
        else assert(out[0] <= prev);
        if (n == AUDIO_PWM_RAMP_SAMPLES) assert(out[0] == AUDIO_PWM_DUTY_MID_Q4);
        prev = out[0];
    }
    assert(n == 2U * AUDIO_PWM_RAMP_SAMPLES);
    assert(out[0] == 0U && reads == 1U);
    done_is_sticky();
}

/* Random traffic: the queue is only read in PLAY, ramps stay within their
 * step bound, and a stop always ends in DONE at 0 V. */
static void random_streams(void)
{
    uint32_t seed = 0x2545f491U;
    for (unsigned run = 0; run < 200; run++) {
        seed = seed * 1664525U + 1013904223U;
        start((seed >> 7) & 1U);
        const unsigned stop_at = (seed >> 8) % 4000U;
        const unsigned bound = step_bound(255U << AUDIO_PWM_DUTY_FRAC_BITS, AUDIO_PWM_RAMP_SAMPLES);
        uint16_t prev[2] = {0, 0};
        unsigned n = 0;
        while (state.shaper.phase != AUDIO_PWM_PHASE_DONE) {
            seed = seed * 1664525U + 1013904223U;
            if (n < stop_at && (seed >> 24) < 40U && queued() < sizeof(queue) - 64U) {
                for (unsigned i = (seed >> 4) % 32U; i > 0; i--)
                    push((uint8_t)(seed >> (i % 16U)), (uint8_t)(seed >> 9));
            }
            if (n == stop_at) state.shaper.stop = true;
            const uint8_t phase = state.shaper.phase;
            const unsigned reads_before = reads;
            (void)tick();
            ++n;
            assert(n < stop_at + sizeof(queue) + 2U * AUDIO_PWM_RAMP_SAMPLES + 2U);
            if (phase != AUDIO_PWM_PHASE_PLAY) assert(reads == reads_before);
            const bool played = reads != reads_before && state.shaper.phase == AUDIO_PWM_PHASE_PLAY;
            for (unsigned c = 0; c < (state.stereo ? 2U : 1U); c++) {
                if (!played) assert(diff(out[c], prev[c]) <= bound);
                prev[c] = out[c];
            }
        }
        assert(out[0] == 0U && (!state.stereo || out[1] == 0U));
        assert(state.shaper.ticks == n);
    }
}

int main(void)
{
    ramp_helper();
    for (unsigned stereo = 0; stereo <= 1U; stereo++) {
        play_underrun_and_drain(stereo);
        stop_during_ramp_up(stereo);
    }
    random_streams();
    puts("PWM audio: streams ramp up and down without steps");
    return 0;
}
