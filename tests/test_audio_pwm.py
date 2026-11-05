from pathlib import Path
import re
import subprocess
import tempfile
import unittest

from test_foreground_presenter import production_functions

ROOT = Path(__file__).resolve().parents[1]


class AudioPwmTest(unittest.TestCase):
    def test_block_pacing_preserves_samples_without_underruns(self):
        source = (ROOT / "src/drivers/audio_pwm.c").read_text()
        with tempfile.TemporaryDirectory() as tmp:
            Path(tmp, "audio_pwm_write.inc").write_text(production_functions(
                source, ("audio_pwm_write_s16",)))
            for tick_hz in (1000, 100):
                binary = Path(tmp, f"audio-pwm-{tick_hz}")
                subprocess.run([
                    "cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-fsanitize=address,undefined", f"-DTICK_HZ={tick_hz}",
                    f"-I{tmp}", str(ROOT / "tests/host/audio_pwm_pacing_test.c"),
                    "-o", str(binary),
                ], check=True)
                subprocess.run([str(binary)], check=True)

    def test_streams_ramp_up_and_down_without_steps(self):
        source = (ROOT / "src/drivers/audio_pwm.c").read_text()
        header = (ROOT / "src/drivers/audio_pwm.h").read_text()
        constants = re.findall(
            r"^#define AUDIO_PWM_(?:DUTY|RAMP|PHASE|CLOSE|USE)_\w+ .+$", source, re.M)
        constants += re.findall(r"^#define AUDIO_PWM_SAMPLE_RATE .+$", header, re.M)
        shaper_type = re.search(
            r"^typedef struct \{[^{}]*\} audio_pwm_shaper_t;", source, re.M)
        self.assertIsNotNone(shaper_type)
        shaper = "\n".join(constants + [
            shaper_type.group(0),
            production_functions(source, ("audio_pwm_ramp_q4", "audio_pwm_shaper_step")),
        ])
        isr = production_functions(source, ("audio_pwm_on_sample",))
        with tempfile.TemporaryDirectory(prefix="solaros-audio-pwm-") as tmp:
            Path(tmp, "audio_pwm_shaper.inc").write_text(shaper + "\n")
            Path(tmp, "audio_pwm_isr.inc").write_text(isr + "\n")
            binary = Path(tmp, "shaper")
            subprocess.run([
                "cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
                "-DIRAM_ATTR=", "-I", tmp,
                str(ROOT / "tests/host/audio_pwm_shaper_test.c"),
                "-o", str(binary),
            ], check=True)
            subprocess.run([str(binary)], check=True)

    def test_open_close_and_attach_keep_the_pins_low(self):
        driver = (ROOT / "src/drivers/audio_pwm.c").read_text()
        service = (ROOT / "src/services/solar_os_audio_pwm.c").read_text()
        # A channel starts at 0 V and the interrupt ramps it up.
        self.assertIn(".duty = 0,", production_functions(driver, ("audio_pwm_open_pins",)))
        # Nothing jumps to the midpoint on close or on an underrun.
        for name in ("audio_pwm_close", "audio_pwm_wait_ramp_down", "audio_pwm_on_sample"):
            with self.subTest(name=name):
                self.assertNotIn("AUDIO_PWM_DUTY_MIDPOINT", production_functions(driver, (name,)))
        self.assertIn("audio_pwm_park_pins(", production_functions(driver, ("audio_pwm_close",)))
        # Fractional duty stays a compile-time switch that defaults on.
        self.assertRegex(driver, r"#ifndef AUDIO_PWM_USE_FRAC_DUTY\n"
                                 r"#define AUDIO_PWM_USE_FRAC_DUTY 1\n#endif")
        # Attach holds the pins low; detach lets go of the hold.
        self.assertIn("audio_pwm_park_pins(",
                      production_functions(service, ("solar_os_audio_pwm_attach",)))
        self.assertIn("audio_pwm_release_pins(",
                      production_functions(service, ("solar_os_audio_pwm_detach",)))


if __name__ == "__main__":
    unittest.main()
