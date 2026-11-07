#include "Epd47Driver.h"

#include <BoardConfig.h>

#include <cstring>

#if FREEINK_DRIVER_EPD47
#include <epd_driver.h>
#include <esp_heap_caps.h>
#endif

namespace freeink {

#if FREEINK_DRIVER_EPD47
namespace {

// The glass, as LilyGo-EPD47 addresses it.
constexpr uint16_t kPanelW = EPD_WIDTH;   // 960
constexpr uint16_t kPanelH = EPD_HEIGHT;  // 540
constexpr size_t kPackedBytes = static_cast<size_t>(kPanelW) * kPanelH / 2;

// 4-bit gray levels in LilyGo-EPD47's packing: even X in the low nibble, odd X
// in the high one. 0xF is white; drawing only ever darkens towards 0x0.
constexpr uint8_t kWhite = 0xF;
constexpr uint8_t kBlack = 0x0;
constexpr uint8_t kGrayLight = 0xA;
constexpr uint8_t kGrayDark = 0x5;

uint16_t g_w = 0, g_h = 0, g_wb = 0;  // logical frame (the X4 Pro's 800x480)
uint16_t g_offX = 0, g_offY = 0;      // where it sits on the glass
uint8_t* g_base = nullptr;            // last B/W frame as 4-bit gray, whole glass
uint8_t* g_gray = nullptr;            // gray-only overlay, whole glass
uint8_t* g_lsb = nullptr;
uint8_t* g_msb = nullptr;

uint8_t* allocPsram(size_t size) {
  auto* p = static_cast<uint8_t*>(heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  return p ? p : static_cast<uint8_t*>(malloc(size));
}

Rect_t glass() { return Rect_t{.x = 0, .y = 0, .width = kPanelW, .height = kPanelH}; }

inline void setPixel(uint8_t* buf, uint16_t px, uint16_t py, uint8_t level) {
  uint8_t& b = buf[(static_cast<uint32_t>(py) * kPanelW + px) >> 1];
  b = (px & 1) ? static_cast<uint8_t>((b & 0x0F) | (level << 4)) : static_cast<uint8_t>((b & 0xF0) | level);
}

// Expand the 1-bpp frame (1 = white) into the centred window of g_base.
void fillBase(const uint8_t* fb) {
  for (uint16_t y = 0; y < g_h; ++y) {
    const uint8_t* src = fb + static_cast<uint32_t>(y) * g_wb;
    uint8_t* dst = g_base + ((static_cast<uint32_t>(y + g_offY) * kPanelW + g_offX) >> 1);
    for (uint16_t bx = 0; bx < g_wb; ++bx) {
      const uint8_t b = src[bx];
      for (uint8_t bit = 0; bit < 8; bit += 2) {
        const uint8_t left = (b & (0x80 >> bit)) ? kWhite : kBlack;
        const uint8_t right = (b & (0x40 >> bit)) ? kWhite : kBlack;
        *dst++ = static_cast<uint8_t>(left | (right << 4));
      }
    }
  }
}

// Fewer clear cycles for faster modes; a full refresh scrubs ghosting.
void clearFor(RefreshMode mode) {
  switch (mode) {
    case RefreshMode::Full: epd_clear(); break;
    case RefreshMode::Half: epd_clear_area_cycles(glass(), 2, 45); break;
    default: epd_clear_area_cycles(glass(), 1, 30); break;
  }
}

}  // namespace
#endif  // FREEINK_DRIVER_EPD47

PanelGeometry Epd47Driver::geometry() const {
  const uint16_t w = BoardConfig::ACTIVE.displayWidth;
  const uint16_t h = BoardConfig::ACTIVE.displayHeight;
  const uint16_t wb = w / 8;
  return {w, h, wb, static_cast<uint32_t>(wb) * h};
}

void Epd47Driver::begin(EpdBus& bus) {
  (void)bus;
#if FREEINK_DRIVER_EPD47
  g_w = BoardConfig::ACTIVE.displayWidth;
  g_h = BoardConfig::ACTIVE.displayHeight;
  g_wb = g_w / 8;
  // Even offset keeps every logical byte on whole packed bytes.
  g_offX = static_cast<uint16_t>(((kPanelW - g_w) / 2) & ~1u);
  g_offY = static_cast<uint16_t>((kPanelH - g_h) / 2);
  if (!g_base) g_base = allocPsram(kPackedBytes);
  if (!g_gray) g_gray = allocPsram(kPackedBytes);
  const size_t plane = static_cast<size_t>(g_wb) * g_h;
  if (!g_lsb) g_lsb = allocPsram(plane);
  if (!g_msb) g_msb = allocPsram(plane);
  if (g_base) memset(g_base, 0xFF, kPackedBytes);
  epd_init();
#endif
}

void Epd47Driver::display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) {
  (void)bus;
  (void)prev;
  (void)turnOff;  // the rails go off after every refresh anyway
#if FREEINK_DRIVER_EPD47
  if (!g_base || !fb) return;
  fillBase(fb);
  epd_poweron();
  clearFor(mode);
  epd_draw_grayscale_image(glass(), g_base);
  epd_poweroff_all();
#else
  (void)fb;
  (void)mode;
#endif
}

void Epd47Driver::copyGrayscaleLsb(EpdBus& bus, const uint8_t* lsb) {
  (void)bus;
#if FREEINK_DRIVER_EPD47
  if (g_lsb && lsb) memcpy(g_lsb, lsb, static_cast<size_t>(g_wb) * g_h);
#else
  (void)lsb;
#endif
}

void Epd47Driver::copyGrayscaleMsb(EpdBus& bus, const uint8_t* msb) {
  (void)bus;
#if FREEINK_DRIVER_EPD47
  if (g_msb && msb) memcpy(g_msb, msb, static_cast<size_t>(g_wb) * g_h);
#else
  (void)msb;
#endif
}

void Epd47Driver::writeGrayscalePlaneStrip(EpdBus& bus, GrayPlane plane, const uint8_t* rows, uint16_t yStart,
                                           uint16_t numRows) {
  (void)bus;
#if FREEINK_DRIVER_EPD47
  uint8_t* dst = (plane == GrayPlane::Lsb) ? g_lsb : g_msb;
  if (!dst || !rows) return;
  memcpy(dst + static_cast<uint32_t>(yStart) * g_wb, rows, static_cast<size_t>(numRows) * g_wb);
#else
  (void)plane;
  (void)rows;
  (void)yStart;
  (void)numRows;
#endif
}

// fb is not the page here (the host leaves its last plane in it, see
// LgfxEpdDriver's overlayCanvasGray). The B/W page is already on the glass, and
// drawing only darkens, so push an image that is white everywhere except the
// pixels the planes select: no clear, no flash.
void Epd47Driver::displayGray(EpdBus& bus, const uint8_t* fb, bool turnOff, const unsigned char* lut,
                              bool factoryMode) {
  (void)bus;
  (void)fb;
  (void)turnOff;
  (void)lut;
  (void)factoryMode;
#if FREEINK_DRIVER_EPD47
  if (!g_gray || !g_lsb || !g_msb) return;
  memset(g_gray, 0xFF, kPackedBytes);
  bool any = false;
  for (uint16_t y = 0; y < g_h; ++y) {
    const uint8_t* lrow = g_lsb + static_cast<uint32_t>(y) * g_wb;
    const uint8_t* mrow = g_msb + static_cast<uint32_t>(y) * g_wb;
    for (uint16_t bx = 0; bx < g_wb; ++bx) {
      const uint8_t l = lrow[bx], m = mrow[bx];
      if ((l | m) == 0) continue;
      for (uint8_t bit = 0; bit < 8; ++bit) {
        const uint8_t mask = 0x80 >> bit;
        const bool lb = (l & mask) != 0, mb = (m & mask) != 0;
        if (!lb && !mb) continue;
        const uint8_t level = (mb && !lb) ? kGrayLight : kGrayDark;
        setPixel(g_gray, g_offX + bx * 8 + bit, g_offY + y, level);
        if (g_base) setPixel(g_base, g_offX + bx * 8 + bit, g_offY + y, level);
        any = true;
      }
    }
  }
  if (!any) return;
  epd_poweron();
  epd_draw_grayscale_image(glass(), g_gray);
  epd_poweroff_all();
#endif
}

void Epd47Driver::cleanupGrayscaleBuffers(EpdBus& bus, const uint8_t* bw) {
  (void)bus;
#if FREEINK_DRIVER_EPD47
  if (bw && g_base) fillBase(bw);
#else
  (void)bw;
#endif
}

void Epd47Driver::deepSleep(EpdBus& bus) {
  (void)bus;
#if FREEINK_DRIVER_EPD47
  epd_poweroff_all();
#endif
}

PanelDriver& epd47Driver() {
  static Epd47Driver instance;
  return instance;
}

}  // namespace freeink
