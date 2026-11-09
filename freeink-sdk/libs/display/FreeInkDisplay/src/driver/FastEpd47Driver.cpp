#include "FastEpd47Driver.h"

#include <BoardConfig.h>

#include <cstring>

#if FREEINK_DRIVER_EPD47_FASTEPD
#include <FastEPD.h>
#include <esp_heap_caps.h>
#endif

namespace freeink {

#if FREEINK_DRIVER_EPD47_FASTEPD
namespace {

constexpr uint16_t kPanelW = 960;
constexpr uint16_t kPanelH = 540;
constexpr uint16_t kPitch1 = kPanelW / 8;  // FastEPD 1 bpp: MSB = leftmost pixel, 1 = white
constexpr uint16_t kPitch4 = kPanelW / 2;  // FastEPD 4 bpp: high nibble = left pixel

// Passes that take a pixel all the way to black or white (FastEPD's default
// partial update uses 4).
#ifndef FREEINK_EPD47_FASTEPD_PASSES
#define FREEINK_EPD47_FASTEPD_PASSES 4
#endif
// Darken pulses for the overlay's two grays, starting from white.
#ifndef FREEINK_EPD47_FASTEPD_DARK_PULSES
#define FREEINK_EPD47_FASTEPD_DARK_PULSES 2
#endif
#ifndef FREEINK_EPD47_FASTEPD_LIGHT_PULSES
#define FREEINK_EPD47_FASTEPD_LIGHT_PULSES 1
#endif
// A gray selection filling at least this share of its bounding box is an
// image (cleared and redrawn); below it, anti-aliased text (no clear).
#ifndef FREEINK_EPD47_IMAGE_DENSITY_PCT
#define FREEINK_EPD47_IMAGE_DENSITY_PCT 30
#endif

constexpr int kFull = FREEINK_EPD47_FASTEPD_PASSES;
constexpr int kDark = FREEINK_EPD47_FASTEPD_DARK_PULSES;
constexpr int kLight = FREEINK_EPD47_FASTEPD_LIGHT_PULSES;
constexpr int kPasses = kFull + (kDark > kLight ? kDark : kLight);

// The 4-bit frame does not hold gray levels here but transitions: FastEPD's
// matrix maps each level to a pulse per pass, so a level can mean "white to
// dark gray" as well as "black to dark gray". One 4-bit pass with no clear then
// moves every pixel of a page, B/W and grays alike, from what is on the glass
// to what should be.
enum Transition : uint8_t {
  kToBlack = 0x0,
  kToWhite = 0x1,
  kDarkFromWhite = 0x2,
  kLightFromWhite = 0x3,
  kDarkFromBlack = 0x4,  // lighten first, then darken: from black or a gray
  kLightFromBlack = 0x5,
  kKeep = 0xF,
};
// 16 levels x passes: 0 = leave, 1 = darken, 2 = lighten. Pulses end on the
// last pass, so every pixel settles together.
uint8_t g_matrix[16 * kPasses];

FASTEPD g_epd;
bool g_ready = false;

// Raised around every FastEPD update: GPIO0 is the register strobe and the
// BOOT key, and the input layer holds the key's state while this is set.
volatile bool g_strobeBusy = false;
struct StrobeBusy {
  StrobeBusy() { g_strobeBusy = true; }
  ~StrobeBusy() { g_strobeBusy = false; }
};

uint16_t g_w = 0, g_h = 0, g_wb = 0;  // logical frame
uint16_t g_offXb = 0, g_offY = 0;     // where it sits on the glass (bytes / rows)

uint8_t* g_prev = nullptr;      // logical frame on the glass, 1 = white
uint8_t* g_grayMask = nullptr;  // logical pixels showing a gray, whatever g_prev says
bool g_havePrev = false;        // g_prev matches the glass (set by any completed refresh)
uint8_t* g_lsb = nullptr;
uint8_t* g_msb = nullptr;

// Combined grayscale base, held from displayGrayscaleBase() to displayGray().
uint8_t* g_pending = nullptr;
bool g_hasPending = false;
RefreshMode g_pendingMode = RefreshMode::Fast;

uint8_t* allocPsram(size_t size) {
  auto* p = static_cast<uint8_t*>(heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  return p ? p : static_cast<uint8_t*>(malloc(size));
}

size_t planeBytes() { return static_cast<size_t>(g_wb) * g_h; }

void setPulses(Transition level, int lighten, int darken) {
  uint8_t* row = g_matrix + level * kPasses;
  // Darken last; a lighten run sits just before it (for the FromBlack grays)
  // or at the end on its own.
  const int darkStart = kPasses - darken;
  const int lightStart = darkStart - lighten;
  for (int pass = 0; pass < kPasses; ++pass) row[pass] = pass >= darkStart ? 1 : pass >= lightStart ? 2 : 0;
}

void buildMatrix() {
  memset(g_matrix, 0, sizeof(g_matrix));  // every unused level, and kKeep, leaves the pixel
  setPulses(kToBlack, 0, kFull);
  setPulses(kToWhite, kFull, 0);
  setPulses(kDarkFromWhite, 0, kDark);
  setPulses(kLightFromWhite, 0, kLight);
  setPulses(kDarkFromBlack, kFull, kDark);
  setPulses(kLightFromBlack, kFull, kLight);
}

bool use1bpp() {
  if (g_epd.getMode() == BB_MODE_1BPP) return true;
  return g_epd.setMode(BB_MODE_1BPP) == BBEP_SUCCESS;
}

bool use4bpp() {
  if (g_epd.getMode() != BB_MODE_4BPP && g_epd.setMode(BB_MODE_4BPP) != BBEP_SUCCESS) return false;
  return g_epd.setCustomMatrix(g_matrix, sizeof(g_matrix)) == BBEP_SUCCESS;
}

// Copy a logical 1-bpp frame into a whole-glass 1-bpp buffer, white margin.
void place1(uint8_t* dst, const uint8_t* src) {
  if (g_w != kPanelW || g_h != kPanelH) memset(dst, 0xFF, static_cast<size_t>(kPitch1) * kPanelH);
  for (uint16_t y = 0; y < g_h; ++y)
    memcpy(dst + static_cast<uint32_t>(y + g_offY) * kPitch1 + g_offXb, src + static_cast<uint32_t>(y) * g_wb, g_wb);
}

// Record the glass: base (1 = white) plus the pixels a gray sits on.
void remember(const uint8_t* base, const uint8_t* gray) {
  memcpy(g_prev, base, planeBytes());
  if (gray) {
    memcpy(g_grayMask, gray, planeBytes());
  } else {
    memset(g_grayMask, 0, planeBytes());
  }
  g_havePrev = true;
}

// Differential B/W update to fb, for pages without grays. FastEPD diffs current
// against previous; the previous plane is rebuilt from what this driver knows
// is on the glass, every gray pixel set to the opposite of its new value so it
// is driven whichever way it goes. (FastEPD's own previous plane shares memory
// with the 4-bit frame, so it cannot be trusted after a gray pass anyway.)
bool partial(const uint8_t* fb) {
  if (!g_havePrev || !use1bpp()) return false;
  place1(g_epd.currentBuffer(), fb);
  uint8_t* prev = g_epd.previousBuffer();
  if (g_w != kPanelW || g_h != kPanelH) memset(prev, 0xFF, static_cast<size_t>(kPitch1) * kPanelH);
  for (uint16_t y = 0; y < g_h; ++y) {
    const uint32_t row = static_cast<uint32_t>(y) * g_wb;
    uint8_t* out = prev + static_cast<uint32_t>(y + g_offY) * kPitch1 + g_offXb;
    for (uint16_t bx = 0; bx < g_wb; ++bx) {
      const uint8_t g = g_grayMask[row + bx];
      out[bx] = static_cast<uint8_t>((g_prev[row + bx] & ~g) | (~fb[row + bx] & g));
    }
  }
  g_epd.setPreviousMode(BB_MODE_1BPP);  // no conversion of the plane we just built
  StrobeBusy busy;
  return g_epd.partialUpdate(false) == BBEP_SUCCESS;
}

// Clearing B/W update: the cleaner, flashing path for Half/Full.
void full(const uint8_t* fb, RefreshMode mode) {
  if (!use1bpp()) return;
  place1(g_epd.currentBuffer(), fb);
  StrobeBusy busy;
  g_epd.fullUpdate(mode == RefreshMode::Full ? CLEAR_SLOW : CLEAR_FAST, false);
}

inline void put4(uint8_t* buf, uint16_t px, uint16_t py, uint8_t level) {
  uint8_t& b = buf[static_cast<uint32_t>(py) * kPitch4 + (px >> 1)];
  b = (px & 1) ? static_cast<uint8_t>((b & 0xF0) | level) : static_cast<uint8_t>((b & 0x0F) | (level << 4));
}

// Fill the 4-bit frame with the transitions to base + grays (lsb/msb planes).
// fromWhite: the glass is white (just cleared), else it is g_prev/g_grayMask.
void buildTransitions(const uint8_t* base, bool fromWhite) {
  uint8_t* buf = g_epd.currentBuffer();
  memset(buf, 0xFF, static_cast<size_t>(kPitch4) * kPanelH);  // kKeep
  const uint16_t offX = g_offXb * 8;
  for (uint16_t y = 0; y < g_h; ++y) {
    const uint32_t row = static_cast<uint32_t>(y) * g_wb;
    for (uint16_t bx = 0; bx < g_wb; ++bx) {
      const uint8_t l = g_lsb[row + bx], m = g_msb[row + bx], sel = l | m;
      const uint8_t oldGray = fromWhite ? 0 : g_grayMask[row + bx];
      const uint8_t oldWhite = fromWhite ? 0xFF : static_cast<uint8_t>(g_prev[row + bx] & ~oldGray);
      const uint8_t newWhite = static_cast<uint8_t>(base[row + bx] & ~sel);
      const uint8_t newBlack = static_cast<uint8_t>(~base[row + bx] & ~sel);
      const uint8_t toWhite = static_cast<uint8_t>(newWhite & ~oldWhite);  // from black or a gray
      const uint8_t toBlack = static_cast<uint8_t>(newBlack & (oldWhite | oldGray));
      if ((toWhite | toBlack | sel) == 0) continue;
      for (uint8_t bit = 0; bit < 8; ++bit) {
        const uint8_t mask = 0x80 >> bit;
        uint8_t level;
        if (sel & mask) {
          const bool light = (m & mask) && !(l & mask);
          if (oldWhite & mask) level = light ? kLightFromWhite : kDarkFromWhite;
          else level = light ? kLightFromBlack : kDarkFromBlack;
        } else if (toWhite & mask) {
          level = kToWhite;
        } else if (toBlack & mask) {
          level = kToBlack;
        } else {
          continue;
        }
        put4(buf, offX + bx * 8 + bit, g_offY + y, level);
      }
    }
  }
}

// base + the planes' grays in one 4-bit pass. clear: CLEAR_NONE moves pixels
// from what is on the glass; otherwise that clear first, within box if given.
void grayUpdate(const uint8_t* base, int clear, BB_RECT* box) {
  if (!use4bpp()) return;
  buildTransitions(base, clear != CLEAR_NONE);
  StrobeBusy busy;
  g_epd.fullUpdate(clear, false, box);
}

}  // namespace
#endif  // FREEINK_DRIVER_EPD47_FASTEPD

PanelGeometry FastEpd47Driver::geometry() const {
  const uint16_t w = BoardConfig::ACTIVE.displayWidth;
  const uint16_t h = BoardConfig::ACTIVE.displayHeight;
  const uint16_t wb = w / 8;
  return {w, h, wb, static_cast<uint32_t>(wb) * h};
}

void FastEpd47Driver::begin(EpdBus& bus) {
  (void)bus;
#if FREEINK_DRIVER_EPD47_FASTEPD
  g_w = BoardConfig::ACTIVE.displayWidth;
  g_h = BoardConfig::ACTIVE.displayHeight;
  g_wb = g_w / 8;
  g_offXb = static_cast<uint16_t>((kPanelW - g_w) / 16);  // whole bytes
  g_offY = static_cast<uint16_t>((kPanelH - g_h) / 2);
  const size_t plane = planeBytes();
  if (!g_prev) g_prev = allocPsram(plane);
  if (!g_grayMask) g_grayMask = allocPsram(plane);
  if (!g_lsb) g_lsb = allocPsram(plane);
  if (!g_msb) g_msb = allocPsram(plane);
  if (!g_pending) g_pending = allocPsram(plane);
  g_hasPending = false;
  g_havePrev = false;  // the glass is unknown until the first clearing refresh
  buildMatrix();
  if (!g_ready) {
    // The predefined panel carries its size; initPanel() allocates the
    // buffers, and a later setPanelSize() would be refused.
    StrobeBusy busy;
    g_ready = g_epd.initPanel(BB_PANEL_LILYGO_T5S3) == BBEP_SUCCESS && g_epd.setMode(BB_MODE_1BPP) == BBEP_SUCCESS;
    if (g_ready) g_epd.setPasses(kFull);
  }
#endif
}

void FastEpd47Driver::display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) {
  (void)bus;
  (void)prev;
  (void)turnOff;  // the rails go off after every refresh anyway
#if FREEINK_DRIVER_EPD47_FASTEPD
  if (!g_ready || !g_prev || !fb) return;
  g_hasPending = false;  // a plain refresh supersedes a held grayscale base
  if (!(mode == RefreshMode::Fast && partial(fb))) full(fb, mode);
  remember(fb, nullptr);
#else
  (void)fb;
  (void)mode;
#endif
}

void FastEpd47Driver::copyGrayscaleLsb(EpdBus& bus, const uint8_t* lsb) {
  (void)bus;
#if FREEINK_DRIVER_EPD47_FASTEPD
  if (g_lsb && lsb) memcpy(g_lsb, lsb, planeBytes());
#else
  (void)lsb;
#endif
}

void FastEpd47Driver::copyGrayscaleMsb(EpdBus& bus, const uint8_t* msb) {
  (void)bus;
#if FREEINK_DRIVER_EPD47_FASTEPD
  if (g_msb && msb) memcpy(g_msb, msb, planeBytes());
#else
  (void)msb;
#endif
}

void FastEpd47Driver::writeGrayscalePlaneStrip(EpdBus& bus, GrayPlane plane, const uint8_t* rows, uint16_t yStart,
                                               uint16_t numRows) {
  (void)bus;
#if FREEINK_DRIVER_EPD47_FASTEPD
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

// fb is not the page here (see Epd47Driver::displayGray); the planes are.
void FastEpd47Driver::displayGray(EpdBus& bus, const uint8_t* fb, bool turnOff, const unsigned char* lut,
                                  bool factoryMode) {
  (void)bus;
  (void)fb;
  (void)turnOff;
  (void)lut;
  (void)factoryMode;
#if FREEINK_DRIVER_EPD47_FASTEPD
  if (!g_ready || !g_lsb || !g_msb || !g_prev || !g_pending) return;
  const uint32_t plane = planeBytes();
  uint8_t* base = g_pending;  // the page's B/W base
  RefreshMode mode = RefreshMode::Fast;
  if (g_hasPending) {
    g_hasPending = false;
    mode = g_pendingMode;
  } else {
    // Grays on top of what is already up (no held base): the base is the glass.
    memcpy(base, g_prev, plane);
  }

  // What the selection covers: its logical bounds, how dense it is, and whether
  // any of it sits on black base pixels.
  bool any = false, selectsBlack = false;
  uint32_t selected = 0;
  uint16_t x0 = g_w, y0 = g_h, x1 = 0, y1 = 0;
  for (uint16_t y = 0; y < g_h; ++y) {
    const uint32_t row = static_cast<uint32_t>(y) * g_wb;
    for (uint16_t bx = 0; bx < g_wb; ++bx) {
      const uint8_t sel = g_lsb[row + bx] | g_msb[row + bx];
      if (!sel) continue;
      any = true;
      selectsBlack |= (sel & ~base[row + bx]) != 0;
      selected += __builtin_popcount(sel);
      if (bx * 8 < x0) x0 = bx * 8;
      if (bx * 8 + 7 > x1) x1 = bx * 8 + 7;
      if (y < y0) y0 = y;
      y1 = y;
    }
  }

  if (mode != RefreshMode::Fast || !g_havePrev) {
    // Half/Full: one clear, then base and grays together.
    grayUpdate(base, mode == RefreshMode::Full ? CLEAR_SLOW : CLEAR_FAST, nullptr);
  } else if (!any) {
    if (!partial(base)) full(base, RefreshMode::Half);
  } else {
    const uint32_t boxArea = static_cast<uint32_t>(x1 - x0 + 1) * (y1 - y0 + 1);
    const bool image = selectsBlack && selected * 100 >= boxArea * FREEINK_EPD47_IMAGE_DENSITY_PCT;
    if (image) {
      // An image: lighten-then-darken over it would leave residue, so the
      // rest of the page moves first and the image box is cleared and redrawn.
      grayUpdate(base, CLEAR_NONE, nullptr);
      BB_RECT box = {g_offXb * 8 + x0, g_offY + y0, x1 - x0 + 1, y1 - y0 + 1};
      grayUpdate(base, CLEAR_FAST, &box);
    } else {
      // A text page: B/W changes and anti-aliasing in one pass, no flash.
      grayUpdate(base, CLEAR_NONE, nullptr);
    }
  }
  for (uint32_t i = 0; i < plane; ++i) base[i] |= g_lsb[i] | g_msb[i];  // grays count as white base
  remember(base, nullptr);
  for (uint32_t i = 0; i < plane; ++i) g_grayMask[i] = g_lsb[i] | g_msb[i];
#endif
}

void FastEpd47Driver::displayGrayscaleBase(EpdBus& bus, const uint8_t* fb, RefreshMode fallback, bool turnOff) {
#if FREEINK_DRIVER_EPD47_FASTEPD
  if (g_pending && fb) {
    memcpy(g_pending, fb, planeBytes());
    g_pendingMode = fallback;
    g_hasPending = true;
    return;
  }
#endif
  display(bus, fb, nullptr, fallback, turnOff);
}

void FastEpd47Driver::cleanupGrayscaleBuffers(EpdBus& bus, const uint8_t* bw) {
  (void)bus;
  (void)bw;  // g_prev already describes the glass
}

void FastEpd47Driver::deepSleep(EpdBus& bus) {
  (void)bus;
#if FREEINK_DRIVER_EPD47_FASTEPD
  if (g_ready) {
    StrobeBusy busy;
    g_epd.einkPower(0);
  }
#endif
}

#if FREEINK_DRIVER_EPD47_FASTEPD
extern "C" bool freeink_epd_strobe_busy() { return g_strobeBusy; }
#endif

PanelDriver& fastEpd47Driver() {
  static FastEpd47Driver instance;
  return instance;
}

}  // namespace freeink
