"""
PlatformIO pre:/post: script: trim LilyGo-EPD47 to its panel driver and make it
work inside this firmware. Only the lilygo_epd47 env depends on it; elsewhere it
does nothing.

1. library.json: build only the panel driver (epd_driver, ed047tc1,
   i2s_data_bus, rmt_pulse) and drop the library's own dependencies. The rest
   bundles zlib and libjpeg, whose symbols collide with PNGdec's zlib at link
   time, plus touch/font code and Button2/SensorLib/ArduinoJson pins the
   firmware does not use.
2. src/rmt_pulse.c tests ESP_IDF_VERSION_MAJOR before anything has included
   esp_idf_version.h, so the first test reads it as 0 and includes the legacy
   driver/rmt.h while later tests use the IDF 5 rmt_tx.h types. Including the
   version header first makes every test agree.
3. src/epd_driver.c creates two 8 KB-stack tasks in internal RAM for every
   frame of a grayscale draw and waits on them forever. In this firmware the
   second draw after boot (Home) hung there; the cause was not pinned down
   (the internal heap still had a 14 KB block), but with the stacks in PSRAM
   the draw completes. Both tasks only shuffle rows through a queue and never
   touch flash. Both run on core 0: the Arduino loop that polls touch shares
   core 1 with the render task, and a priority-10 busy wait there lost taps.
   That row queue (64 x 480 B) moves to PSRAM too: in internal
   RAM it left the JPEG decoder ~4 KB short, so book images failed to decode
   and showed as black boxes. The 64 KB 4-bit conversion LUT (MALLOC_CAP_8BIT,
   which lands in internal RAM first) goes to PSRAM as well: with it inside,
   esp_wifi_init failed with ESP_ERR_NO_MEM and no network was ever found.
4. src/epd_driver.c: epd_draw_frame_1bit only ever darkens (each set bit
   becomes the 01 push code, whatever the mode). With WHITE_ON_BLACK it now
   shifts the codes to 10, the lighten push, so a differential refresh can
   move pixels both ways without clearing the screen. Its row was also built
   with the two 8-pixel halves of every 16 swapped, unlike the 4-bpp path
   that draws correctly on this S3 bus (USER_I2S_REG 0); it now uses the same
   natural order. Input bit 0 is the leftmost pixel of each byte.

5. src/ed047tc1.c: STR (GPIO0, also the BOOT key) becomes open-drain with
   its pull-up, so the firmware can read the key between register writes.

Registered as pre: (so library.json is trimmed before the library finder reads
it) and post: (to catch a dependency installed during this very build; that
first build then needs one re-run). Like patch_sdfat.py it touches only this
env's copy. A file with none of its edits applied must hash to the reviewed
upstream (pinned commit f0ff4a9); missing edits are then applied only where
their anchor occurs exactly once, and anything else fails the build.
"""

import hashlib
import json
from pathlib import Path

LIBRARY = "LilyGo-EPD47"
JSON_UPSTREAM = "71434d5c6d54b1bf1411cc58c9fdf238ddb69871c54a008479e9825c80c97994"
SRC_FILTER = ["-<*>", "+<epd_driver.c>", "+<ed047tc1.c>", "+<i2s_data_bus.c>", "+<rmt_pulse.c>"]

# (file, reviewed upstream sha256, [(old, new), ...]); a file counts as patched
# when every `new` is present.
SOURCE_PATCHES = (
    ("src/rmt_pulse.c", "6edf74c017c4964104c86efe298fd73597a79bfb20afc78ae2a2a8409e3435fb", [
        (b'#include "rmt_pulse.h"\n', b'#include "rmt_pulse.h"\n#include <esp_idf_version.h>\n'),
    ]),
    # This file has CRLF line endings.
    ("src/epd_driver.c", "6fa4f6c51d4f003bae3a80085dfa91d1d41844d1c39e9f319247dccfc54e8b00", [
        (b"xTaskCreatePinnedToCore((void (*)(void *))provide_out, \"privide_out\", 8192,\r\n"
         b"                                &p1, 10, &t1, 0);",
         b"xTaskCreatePinnedToCoreWithCaps((void (*)(void *))provide_out, \"privide_out\", 8192,\r\n"
         b"                                &p1, 10, &t1, 0, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);"),
        (b"xTaskCreatePinnedToCore((void (*)(void *))feed_display, \"render\", 8192, &p2,\r\n"
         b"                                10, &t2, 1);",
         # Core 0, not 1: feed_display busy-waits at priority 10 and on core 1
         # starved the Arduino loop, which polls the touch panel.
         b"xTaskCreatePinnedToCoreWithCaps((void (*)(void *))feed_display, \"render\", 8192, &p2,\r\n"
         b"                                10, &t2, 0, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);"),
        (b"conversion_lut = (uint8_t *)heap_caps_malloc(1 << 16, MALLOC_CAP_8BIT);",
         b"conversion_lut = (uint8_t *)heap_caps_malloc(1 << 16, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);"),
        (b"output_queue = xQueueCreate(64, EPD_WIDTH / 2);",
         b"output_queue = xQueueCreateWithCaps(64, EPD_WIDTH / 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);"),
        (b"vTaskDelete(t1);\r\n        vTaskDelete(t2);",
         b"vTaskDeleteWithCaps(t1);\r\n        vTaskDeleteWithCaps(t2);"),
        (b"        wide_epd_input[j] = (lut_1bpp[v1] << 16) | lut_1bpp[v2];",
         b"        wide_epd_input[j] = (lut_1bpp[v1] | (lut_1bpp[v2] << 16)) << (mode == WHITE_ON_BLACK ? 1 : 0);"),
    ]),
    # The BOOT key shares GPIO0 with the shift register's STR (store) line.
    # Open-drain with the pull-up: STR still strobes, a pressed key only holds
    # the register's outputs (STR low) and never fights a driven-high pin, and
    # between writes the pin reads back as the key (firmware's button hook).
    # After each release the pull-up needs a moment before the next strobe.
    ("src/ed047tc1.c", "7a2332f538bb5528a3c9f2d3ee58b41e7967c1329751d78586531d425cb86e38", [
        (b"#include <string.h>\n", b"#include <string.h>\n#include <esp_rom_sys.h>\n"),
        (b"    gpio_set_direction(CFG_STR, GPIO_MODE_OUTPUT);\n",
         b"    gpio_set_direction(CFG_STR, GPIO_MODE_INPUT_OUTPUT_OD);\n    gpio_pullup_en(CFG_STR);\n"),
        (b"    push_cfg_bit(cfg->ep_latch_enable);\n\n    fast_gpio_set_hi(CFG_STR);\n}",
         b"    push_cfg_bit(cfg->ep_latch_enable);\n\n    fast_gpio_set_hi(CFG_STR);\n    esp_rom_delay_us(2);\n}"),
    ]),
)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def patch_library_json(dependency):
    path = dependency / "library.json"
    data = path.read_bytes()
    manifest = json.loads(data)
    if manifest.get("build", {}).get("srcFilter") == SRC_FILTER and "dependencies" not in manifest:
        return
    if digest(data) != JSON_UPSTREAM:
        raise RuntimeError("Unrecognized " + LIBRARY + " library.json")
    manifest.pop("dependencies", None)
    manifest["build"] = {"srcFilter": SRC_FILTER}
    path.write_text(json.dumps(manifest, indent=2) + "\n")
    print("Applied " + LIBRARY + " patch: library.json builds the panel driver only")


def patch_source(dependency, relative, upstream, edits):
    path = dependency / relative
    data = path.read_bytes()
    missing = [(old, new) for old, new in edits if new not in data]
    if not missing:
        return
    if len(missing) == len(edits) and digest(data) != upstream:
        raise RuntimeError("Unrecognized " + LIBRARY + " source: " + relative)
    for old, new in missing:
        if data.count(old) != 1:
            raise RuntimeError(LIBRARY + " patch anchor not unique in " + relative)
        data = data.replace(old, new)
    path.write_bytes(data)
    print("Applied " + LIBRARY + " patch: " + relative)


def patch_installed_copy(env):
    if env.get("ARDUINO_LIB_COMPILE_FLAG") == "Build":
        return
    dependency = Path(env.subst("$PROJECT_LIBDEPS_DIR")) / env["PIOENV"] / LIBRARY
    if not (dependency / "library.json").is_file():
        return  # not this env's dependency, or not installed yet
    patch_library_json(dependency)
    for relative, upstream, edits in SOURCE_PATCHES:
        patch_source(dependency, relative, upstream, edits)


if "Import" in globals():
    Import("env")  # noqa: F821 -- supplied by SCons
    patch_installed_copy(env)  # noqa: F821
