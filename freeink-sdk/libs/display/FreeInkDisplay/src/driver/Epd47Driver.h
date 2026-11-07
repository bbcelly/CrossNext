#pragma once

// Raw-parallel ED047TC1 on the PLAIN LilyGo T5-4.7 E-Paper S3 (V2.3/V2.4,
// LilyGo-EPD47 esp32s3), via LilyGo's own LilyGo-EPD47 library.
//
// Not the T5 S3 Pro/Lite that LgfxEpdDriver serves: the plain board has no PMIC
// or PCA9535. Its STV/OE/LE/power lines sit behind a 74HCT4094 shift register
// (data 13, clock 12, store 0), which LovyanGFX's Panel_EPD cannot drive, while
// LilyGo-EPD47 does exactly that.
//
// The board is used as a development stand-in for the 800x480 Xteink X4 Pro, so
// the profile reports 800x480 and this driver centres that frame on the 960x540
// glass. Layouts then match the X4 Pro pixel for pixel; the margin stays white.
//
// Selection: linked only when -DFREEINK_DRIVER_EPD47=1 (derived from
// FREEINK_DEVICE_LILYGO + FREEINK_LILYGO_EPD47), which also needs LilyGo-EPD47 in
// the env's lib_deps.

#include "PanelDriver.h"

namespace freeink {

class Epd47Driver : public PanelDriver {
 public:
  uint32_t spiHz() const override { return 0; }  // LilyGo-EPD47 owns the bus
  BusyPolarity busyPolarity() const override { return BusyPolarity::ActiveLow; }
  bool usesExternalBus() const override { return true; }
  PanelGeometry geometry() const override;

  void begin(EpdBus& bus) override;
  void deepSleep(EpdBus& bus) override;
  void display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) override;

  // Same overlay masks as LgfxEpdDriver: the facade streams LSB/MSB 1-bpp
  // planes and displayGray applies the grays they select.
  GrayscaleCapabilities grayscaleCapabilities(GrayscaleMode mode = GrayscaleMode::Overlay) const override {
    if (mode != GrayscaleMode::Overlay) return {};
    return {GrayscaleEncoding::OverlayMasks, GrayscaleBase::Combined, true, false, false};
  }
  // Combined base: the B/W base is held until displayGray() so a text page goes
  // up in one step instead of bold-then-thin.
  void displayGrayscaleBase(EpdBus& bus, const uint8_t* fb, RefreshMode fallback, bool turnOff) override;
  void copyGrayscaleLsb(EpdBus& bus, const uint8_t* lsb) override;
  void copyGrayscaleMsb(EpdBus& bus, const uint8_t* msb) override;
  void writeGrayscalePlaneStrip(EpdBus& bus, GrayPlane plane, const uint8_t* rows, uint16_t yStart,
                                uint16_t numRows) override;
  void displayGray(EpdBus& bus, const uint8_t* fb, bool turnOff, const unsigned char* lut, bool factoryMode) override;
  void cleanupGrayscaleBuffers(EpdBus& bus, const uint8_t* bw) override;
};

PanelDriver& epd47Driver();

}  // namespace freeink
