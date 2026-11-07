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
// The overlay's two grays. LilyGo-EPD47's 4-bit levels are far from linear:
// 0x5/0xA (an even split) come out much too dark, so they sit higher.
#ifndef FREEINK_EPD47_GRAY_DARK
#define FREEINK_EPD47_GRAY_DARK 0x9
#endif
#ifndef FREEINK_EPD47_GRAY_LIGHT
#define FREEINK_EPD47_GRAY_LIGHT 0xD
#endif
constexpr uint8_t kGrayLight = FREEINK_EPD47_GRAY_LIGHT;
constexpr uint8_t kGrayDark = FREEINK_EPD47_GRAY_DARK;

uint16_t g_w = 0, g_h = 0, g_wb = 0;  // logical frame (the X4 Pro's 800x480)
uint16_t g_offX = 0, g_offY = 0;      // where it sits on the glass
uint8_t* g_base = nullptr;            // last B/W frame as 4-bit gray, whole glass
uint8_t* g_gray = nullptr;            // gray-only overlay, whole glass
uint8_t* g_lsb = nullptr;
uint8_t* g_msb = nullptr;

// Differential (Fast) refresh state, all 1 bpp.
constexpr size_t kMaskBytes = static_cast<size_t>(kPanelW) / 8 * kPanelH;
uint8_t* g_prev = nullptr;      // logical frame now on the glass (1 = white)
uint8_t* g_grayMask = nullptr;  // logical pixels the last gray overlay darkened
uint8_t* g_toBlack = nullptr;   // whole-glass push masks
uint8_t* g_toWhite = nullptr;
bool g_havePrev = false;        // g_prev matches the glass (set by a clearing refresh)

// Push passes and per-row drive time (1/10 us) for a differential refresh.
// More of either gives deeper blacks and cleaner whites but takes longer.
#ifndef FREEINK_EPD47_FAST_PASSES
#define FREEINK_EPD47_FAST_PASSES 4
#endif
#ifndef FREEINK_EPD47_FAST_TIME
#define FREEINK_EPD47_FAST_TIME 400
#endif
// Clear cycles for the area of an image before its gray composite is drawn.
#ifndef FREEINK_EPD47_IMAGE_CLEAR_CYCLES
#define FREEINK_EPD47_IMAGE_CLEAR_CYCLES 2
#endif

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

// LilyGo-EPD47's 1-bpp input takes the leftmost pixel in bit 0; ours is MSB first.
uint8_t g_bitrev[256];
void buildBitrev() {
  for (int i = 0; i < 256; ++i) {
    uint8_t r = 0;
    for (int b = 0; b < 8; ++b) r |= ((i >> b) & 1) << (7 - b);
    g_bitrev[i] = r;
  }
}

// Move only the pixels that differ from what the glass shows, both ways, with
// no clear. Pixels the gray overlay touched count as changed, so anti-aliasing
// does not linger. Returns false when it cannot run (no known glass state).
bool fastRefresh(const uint8_t* fb) {
  if (!g_havePrev || !g_prev || !g_grayMask || !g_toBlack || !g_toWhite) return false;
  memset(g_toBlack, 0, kMaskBytes);
  memset(g_toWhite, 0, kMaskBytes);
  constexpr uint16_t kPanelWb = kPanelW / 8;
  const uint16_t offXb = g_offX / 8;
  bool any = false;
  for (uint16_t y = 0; y < g_h; ++y) {
    const uint32_t row = static_cast<uint32_t>(y) * g_wb;
    const uint32_t out = static_cast<uint32_t>(y + g_offY) * kPanelWb + offXb;
    for (uint16_t bx = 0; bx < g_wb; ++bx) {
      const uint8_t n = fb[row + bx], p = g_prev[row + bx], g = g_grayMask[row + bx];
      const uint8_t black = static_cast<uint8_t>(~n & (p | g));
      const uint8_t white = static_cast<uint8_t>(n & (~p | g));
      g_toBlack[out + bx] = g_bitrev[black];
      g_toWhite[out + bx] = g_bitrev[white];
      any |= (black | white) != 0;
    }
  }
  if (any) {
    epd_poweron();
    for (int pass = 0; pass < FREEINK_EPD47_FAST_PASSES; ++pass) {
      epd_draw_frame_1bit(glass(), g_toBlack, BLACK_ON_WHITE, FREEINK_EPD47_FAST_TIME);
      epd_draw_frame_1bit(glass(), g_toWhite, WHITE_ON_BLACK, FREEINK_EPD47_FAST_TIME);
    }
    epd_poweroff_all();
  }
  return true;
}

// Record fb as what the glass now shows.
void rememberFrame(const uint8_t* fb) {
  if (!g_prev || !g_grayMask) return;
  memcpy(g_prev, fb, static_cast<size_t>(g_wb) * g_h);
  memset(g_grayMask, 0, static_cast<size_t>(g_wb) * g_h);
  g_havePrev = true;
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
  if (!g_prev) g_prev = allocPsram(plane);
  if (!g_grayMask) g_grayMask = allocPsram(plane);
  if (!g_toBlack) g_toBlack = allocPsram(kMaskBytes);
  if (!g_toWhite) g_toWhite = allocPsram(kMaskBytes);
  g_havePrev = false;  // the glass is unknown until the first clearing refresh
  buildBitrev();
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
  if (mode == RefreshMode::Fast && fastRefresh(fb)) {
    rememberFrame(fb);
    return;
  }
  epd_poweron();
  clearFor(mode);
  epd_draw_grayscale_image(glass(), g_base);
  epd_poweroff_all();
  rememberFrame(fb);
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
// LgfxEpdDriver's overlayCanvasGray); the B/W page is already on the glass.
//
// Two cases. Anti-aliased text selects pixels the base left white, and drawing
// only darkens, so an image that is white except the selected pixels goes on
// top: no clear, no flash. Images are drawn black in the base and lightened by
// their planes; pushing those back to white leaves them muddy, so the selected
// area is cleared and the whole composite (base + grays) drawn in one 4-bit
// pass, which is what gives clean gray levels on this panel.
void Epd47Driver::displayGray(EpdBus& bus, const uint8_t* fb, bool turnOff, const unsigned char* lut,
                              bool factoryMode) {
  (void)bus;
  (void)fb;
  (void)turnOff;
  (void)lut;
  (void)factoryMode;
#if FREEINK_DRIVER_EPD47
  if (!g_gray || !g_lsb || !g_msb || !g_base) return;
  memset(g_gray, 0xFF, kPackedBytes);
  bool any = false;
  bool selectsBlack = false;
  uint16_t x0 = g_w, y0 = g_h, x1 = 0, y1 = 0;  // logical bounds of the selection
  for (uint16_t y = 0; y < g_h; ++y) {
    const uint32_t row = static_cast<uint32_t>(y) * g_wb;
    for (uint16_t bx = 0; bx < g_wb; ++bx) {
      const uint8_t l = g_lsb[row + bx], m = g_msb[row + bx];
      if ((l | m) == 0) continue;
      if (g_havePrev && g_prev && ((l | m) & ~g_prev[row + bx])) selectsBlack = true;
      if (bx * 8 < x0) x0 = bx * 8;
      if (bx * 8 + 7 > x1) x1 = bx * 8 + 7;
      if (y < y0) y0 = y;
      y1 = y;
      for (uint8_t bit = 0; bit < 8; ++bit) {
        const uint8_t mask = 0x80 >> bit;
        const bool lb = (l & mask) != 0, mb = (m & mask) != 0;
        if (!lb && !mb) continue;
        const uint8_t level = (mb && !lb) ? kGrayLight : kGrayDark;
        setPixel(g_gray, g_offX + bx * 8 + bit, g_offY + y, level);
        setPixel(g_base, g_offX + bx * 8 + bit, g_offY + y, level);
        if (g_grayMask) g_grayMask[row + bx] |= mask;
        any = true;
      }
    }
  }
  if (!any) return;
  epd_poweron();
  if (selectsBlack) {
    const Rect_t area = {.x = g_offX + x0, .y = g_offY + y0, .width = x1 - x0 + 1, .height = y1 - y0 + 1};
    epd_clear_area_cycles(area, FREEINK_EPD47_IMAGE_CLEAR_CYCLES, 45);
    epd_draw_grayscale_image(glass(), g_base);
  } else {
    epd_draw_grayscale_image(glass(), g_gray);
  }
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
