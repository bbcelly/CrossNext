"""
PlatformIO pre:/post: script: teach FastEPD the PLAIN LilyGo T5-4.7 E-Paper S3
(V2.3/V2.4). Only the lilygo_epd47_fastepd env depends on it; elsewhere it does
nothing.

FastEPD already drives the board's 74HCT4094 control register: its LilyGo*
callbacks (written for the older T5 "Pro" H752) use the same bit layout and
sequencing as the vendor ed047tc1.c. What it lacks, ported from the InkDeck
board port that runs on this hardware (bbcelly/Book32, lilygo-t5s3-plain):

1. FastEPD.h / FastEPD.inl: BB_PANEL_LILYGO_T5S3, the EPD47 pins (D0..D7 =
   8,1,2,3,4,5,6,7, CKV 38, STH 40, CKH 41, register data 13 / clock 12 /
   store 0) with the LilyGo callbacks. The enum, panelDefs and panelProcs are
   indexed together, so the new entry goes right after BB_PANEL_LILYGO_T5P4 in
   all three.
2. FastEPD.inl: on this board STH is an active-high start pulse ahead of the
   row, not a chip select. It is routed to the I80 D/C line and each row goes
   out as a 10-bit zero command (D/C high) followed by the pixel bytes (D/C
   low), the sequence LilyGo's i2s_data_bus.c uses. With STH as a chip select
   (FastEPD's other modes) the panel stays blank. Bus at 10 MHz like the
   vendor driver.
3. FastEPD.inl: STR (GPIO0) is also the BOOT key. Open-drain with the pull-up,
   so a pressed key never fights the driver and the firmware reads the key
   between register writes; each strobe ends with a 2 us settle for the
   pull-up. Same as patch_lilygo_epd47.py does for LilyGo-EPD47.

Registered as pre: and post: like patch_lilygo_epd47.py. A file with none of
its edits applied must hash to the reviewed upstream (pinned commit 60d4945,
FastEPD 2.2.0); missing edits are then applied only where their anchor occurs
exactly once, and anything else fails the build.
"""

import hashlib
from pathlib import Path

LIBRARY = "FastEPD"

SOURCE_PATCHES = (
    ("src/FastEPD.h", "b3d8707ae90007729892d1e4ddd8f8a64e33ec5b00e6bf8cc2367ba731cf594c", [
        (b"    BB_PANEL_LILYGO_T5P4,\n",
         b"    BB_PANEL_LILYGO_T5P4,\n"
         b"    BB_PANEL_LILYGO_T5S3, // plain T5-4.7 E-Paper S3 V2.3/V2.4 (74HCT4094 shift register)\n"),
    ]),
    ("src/FastEPD.inl", "1a58be68b1a2c4ee4e20aab1f2ddf9191aaa4d3b514129064714d71ee73d389d", [
        (b"11 /* LED1_EN */, u8M5Matrix, sizeof(u8M5Matrix), 16, -1600}, // BB_PANEL_LILYGO_T5P4 \n",
         b"11 /* LED1_EN */, u8M5Matrix, sizeof(u8M5Matrix), 16, -1600}, // BB_PANEL_LILYGO_T5P4 \n"
         b"    // Plain LilyGo T5-4.7 E-Paper S3 V2.3/V2.4: the H752's register layout on the EPD47 pins.\n"
         b"    // STH (40) is the I80 D/C line, see bbepIOInit; GPIO48 stays an unused dummy.\n"
         b"    {960, 540, 10000000, BB_PANEL_FLAG_NONE, {8,1,2,3,4,5,6,7}, 8, BB_NOT_USED, BB_NOT_USED, 38, 40, 0, 0,\n"
         b"      41, 0, 13, 12, 0, 0, 48, u8M5Matrix, sizeof(u8M5Matrix), 16, -1600}, // BB_PANEL_LILYGO_T5S3\n"),
        (b"EPDiyV7ExtIO}, // BB_PANEL_LILYGO_T5P4  \n",
         b"EPDiyV7ExtIO}, // BB_PANEL_LILYGO_T5P4  \n"
         b"    {LilyGoEinkPower, LilyGoIOInit, LilyGoRowControl, NULL, NULL}, // BB_PANEL_LILYGO_T5S3\n"),
        (b"static uint8_t bSlowSPH = 0;\n",
         b"static uint8_t bSlowSPH = 0;\n"
         b"// Plain LilyGo T5-4.7 S3: STH is clocked in as a 10-bit zero command on D/C.\n"
         b"static uint8_t bSthCommand = 0;\n"),
        (b"esp_lcd_panel_io_tx_color(io_handle, -1, pData,",
         b"esp_lcd_panel_io_tx_color(io_handle, bSthCommand ? 0 : -1, pData,"),
        (b"    s3_bus_config.dc_gpio_num = (gpio_num_t)pState->panelDef.ioDCDummy;\n",
         b"    s3_bus_config.dc_gpio_num = (gpio_num_t)((pState->iPanelType == BB_PANEL_LILYGO_T5S3)\n"
         b"                                             ? pState->panelDef.ioSPH : pState->panelDef.ioDCDummy);\n"),
        (b"    if (pState->panelDef.flags & BB_PANEL_FLAG_SLOW_SPH) {\n        bSlowSPH = 1;\n",
         b"    if (pState->iPanelType == BB_PANEL_LILYGO_T5S3) {\n"
         b"        // STH is an active-high start pulse ahead of the row, not a chip\n"
         b"        // select: D/C (routed to STH above) goes high for a 10-bit zero\n"
         b"        // command, then low for the pixel bytes.\n"
         b"        bSthCommand = 1;\n"
         b"        bSlowSPH = 0;\n"
         b"        s3_io_config.cs_gpio_num = (gpio_num_t)-1;\n"
         b"        s3_io_config.lcd_cmd_bits = 10;\n"
         b"        s3_io_config.dc_levels.dc_idle_level = 0;\n"
         b"        s3_io_config.dc_levels.dc_cmd_level = 1;\n"
         b"        s3_io_config.dc_levels.dc_dummy_level = 0;\n"
         b"        s3_io_config.dc_levels.dc_data_level = 0;\n"
         b"    } else if (pState->panelDef.flags & BB_PANEL_FLAG_SLOW_SPH) {\n        bSlowSPH = 1;\n"),
        (b"    gpio_set_level((gpio_num_t)pState->panelDef.ioShiftSTR, 1);\n} /* bbepSendShiftData() */",
         b"    gpio_set_level((gpio_num_t)pState->panelDef.ioShiftSTR, 1);\n"
         b"    // Plain T5-4.7 S3: STR (GPIO0) is open-drain; let the pull-up raise it.\n"
         b"    if (pState->iPanelType == BB_PANEL_LILYGO_T5S3) delayMicroseconds(2);\n"
         b"} /* bbepSendShiftData() */"),
        (b"    bbepPinMode(pState->panelDef.ioShiftSTR, OUTPUT);\n",
         b"    if (pState->iPanelType == BB_PANEL_LILYGO_T5S3) {\n"
         b"        // STR shares GPIO0 with the BOOT key (button to GND, pull-up).\n"
         b"        gpio_config_t io_conf = {};\n"
         b"        io_conf.pin_bit_mask = 1ULL << pState->panelDef.ioShiftSTR;\n"
         b"        io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;\n"
         b"        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;\n"
         b"        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;\n"
         b"        io_conf.intr_type = GPIO_INTR_DISABLE;\n"
         b"        gpio_reset_pin((gpio_num_t)pState->panelDef.ioShiftSTR);\n"
         b"        gpio_config(&io_conf);\n"
         b"    } else {\n"
         b"        bbepPinMode(pState->panelDef.ioShiftSTR, OUTPUT);\n"
         b"    }\n"),
    ]),
)


def digest(data):
    return hashlib.sha256(data).hexdigest()


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
    if not (dependency / "library.properties").is_file():
        return  # not this env's dependency, or not installed yet
    for relative, upstream, edits in SOURCE_PATCHES:
        patch_source(dependency, relative, upstream, edits)


if "Import" in globals():
    Import("env")  # noqa: F821 -- supplied by SCons
    patch_installed_copy(env)  # noqa: F821
