#include "solar_os_audio_pwm.h"

static const solar_os_expansion_binding_spec_t binding_specs[] = {
    {.key = "pwm", .value_hint = "gpio", .kind = SOLAR_OS_EXPANSION_BINDING_PWM, .role = "pwm", .required = true},
    {.key = "pwm_right", .value_hint = "gpio", .kind = SOLAR_OS_EXPANSION_BINDING_PWM, .role = "pwm_right"},
};

const solar_os_expansion_driver_t solar_os_audio_pwm_expansion_driver = {
    .name = "audio-pwm",
    .category = SOLAR_OS_EXPANSION_CATEGORY_AUDIO,
    .summary = "PWM audio (mono, or stereo with pwm_right)",
    .required_capabilities = SOLAR_OS_BOARD_CAP_EXPANSION_PWM,
    .binding_specs = binding_specs,
    .binding_spec_count = sizeof(binding_specs) / sizeof(binding_specs[0]),
    .attach = solar_os_audio_pwm_attach,
    .detach = solar_os_audio_pwm_detach,
};
