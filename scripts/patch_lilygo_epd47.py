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
3. src/epd_driver.c creates two 8 KB-stack tasks for every frame of a
   grayscale draw and waits on them forever. Once the reader has opened a book
   the internal heap no longer has 8 KB contiguous, xTaskCreatePinnedToCore
   fails silently and the draw hangs (seen on the second screen after boot).
   The stacks go to PSRAM instead; both tasks only shuffle rows through a
   queue and never touch flash.

Registered as pre: (so library.json is trimmed before the library finder reads
it) and post: (to catch a dependency installed during this very build; that
first build then needs one re-run). Like patch_sdfat.py it touches only this
env's copy, and only files that hash to the reviewed upstream (pinned commit
f0ff4a9) or are already patched; anything else fails the build.
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
         b"xTaskCreatePinnedToCoreWithCaps((void (*)(void *))feed_display, \"render\", 8192, &p2,\r\n"
         b"                                10, &t2, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);"),
        (b"vTaskDelete(t1);\r\n        vTaskDelete(t2);",
         b"vTaskDeleteWithCaps(t1);\r\n        vTaskDeleteWithCaps(t2);"),
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
    if all(new in data for _, new in edits):
        return
    if digest(data) != upstream:
        raise RuntimeError("Unrecognized " + LIBRARY + " source: " + relative)
    for old, new in edits:
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
