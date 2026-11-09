#pragma once

// Raw-parallel ED047TC1 on the PLAIN LilyGo T5-4.7 E-Paper S3 (V2.3/V2.4), via
// bitbank2's FastEPD instead of LilyGo-EPD47 (Epd47Driver).
//
// Same board, same 74HCT4094 control register; FastEPD streams rows through the
// S3's I80 LCD peripheral with DMA and its differential (partial) update takes
// ~4 passes, where Epd47Driver's page turn spent ~1.4 s in LilyGo-EPD47's
// frame loops. FastEPD keeps its buffers in PSRAM (LilyGo-EPD47's internal-RAM
// allocations were this board's recurring failure) and is Apache-2.0, so a
// build with it is distributable. The board entry (pins, STH as the I80 D/C
// start pulse, open-drain STR for the BOOT key) is patched in by
// scripts/patch_fastepd.py.
//
// Pages without grays go up as FastEPD's 1-bit differential update. A page with
// grays (anti-aliased text) is one 4-bit pass with no clear whose levels are
// transitions, not shades: keep, to black, to white, gray from white, gray from
// black. B/W changes and anti-aliasing land together in kFull + 2 passes, with
// no flash. An image (a dense gray selection over black base pixels) also
// clears its box and redraws base + grays there, as Epd47Driver does.
//
// Selection: -DFREEINK_EPD47_FASTEPD=1 on top of FREEINK_LILYGO_EPD47 links this
// driver (FREEINK_DRIVER_EPD47_FASTEPD) instead of Epd47Driver; the env needs
// FastEPD in lib_deps and the patch script.

#include "PanelDriver.h"

namespace freeink {

class FastEpd47Driver : public PanelDriver {
 public:
  uint32_t spiHz() const override { return 0; }  // FastEPD owns the bus
  BusyPolarity busyPolarity() const override { return BusyPolarity::ActiveLow; }
  bool usesExternalBus() const override { return true; }
  PanelGeometry geometry() const override;

  void begin(EpdBus& bus) override;
  void deepSleep(EpdBus& bus) override;
  void display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) override;

  // Same contract as Epd47Driver: overlay masks, combined base.
  GrayscaleCapabilities grayscaleCapabilities(GrayscaleMode mode = GrayscaleMode::Overlay) const override {
    if (mode != GrayscaleMode::Overlay) return {};
    return {GrayscaleEncoding::OverlayMasks, GrayscaleBase::Combined, true, false, false};
  }
  void displayGrayscaleBase(EpdBus& bus, const uint8_t* fb, RefreshMode fallback, bool turnOff) override;
  void copyGrayscaleLsb(EpdBus& bus, const uint8_t* lsb) override;
  void copyGrayscaleMsb(EpdBus& bus, const uint8_t* msb) override;
  void writeGrayscalePlaneStrip(EpdBus& bus, GrayPlane plane, const uint8_t* rows, uint16_t yStart,
                                uint16_t numRows) override;
  void displayGray(EpdBus& bus, const uint8_t* fb, bool turnOff, const unsigned char* lut, bool factoryMode) override;
  void cleanupGrayscaleBuffers(EpdBus& bus, const uint8_t* bw) override;
};

PanelDriver& fastEpd47Driver();

}  // namespace freeink
