#pragma once

#include "epaper_spi.h"
#include "epaper_spi_mono.h"

namespace esphome::epaper_spi {

/** Display modes for the SSD1683 e-paper controller. */
enum class SSD1683DisplayMode : uint8_t {
  FULL = 0,        ///< Full refresh using internal waveforms
  PARTIAL = 1,     ///< Partial refresh (partial window update)
  GRAYSCALE4 = 2,  ///< 4-shade grayscale using custom LUT
};

/**
 * Driver for SSD1683 / compatible e-paper displays.
 *
 * Supports monochrome full/partial refresh and 4-shade grayscale.
 *
 * Currently tested with:
 * - Waveshare 4.2in V2 (400×300)
 *
 * Grayscale mode uses a dual-buffer approach: both the black (0x24) and
 * red (0x26) RAM planes are written with complementary 1-bit slices of
 * each 2-bit pixel value, driven by a custom LUT.
 *
 * @internal \\c final saves a few bytes via devirtualisation. Remove if subclassing.
 */
class EPaperSSD1683 final : public EPaperMono {
 public:
  EPaperSSD1683(const char *name, uint16_t width, uint16_t height, const uint8_t *init_sequence,
                size_t init_sequence_length, const uint8_t *gray_lut, size_t gray_lut_length)
      : EPaperMono(name, width, height, init_sequence, init_sequence_length),
        gray_lut_(gray_lut),
        gray_lut_length_(gray_lut_length) {
    // 2bpp grayscale buffer: 4 pixels per byte
    this->gray_row_width_ = (width + 3) / 4;
    this->gray_buffer_length_ = this->gray_row_width_ * height;
  }

  void set_display_mode(SSD1683DisplayMode mode) { this->display_mode_ = mode; }

  /**
   * Override setup() to promote buffer_length_ to grayscale size before EPaperBase
   * allocates the SplitBuffer. set_display_mode() is called before setup().
   */
  void setup() override;

  DisplayType get_display_type() override {
    if (this->display_mode_ == SSD1683DisplayMode::GRAYSCALE4)
      return DISPLAY_TYPE_GRAYSCALE;
    return DISPLAY_TYPE_BINARY;
  }

  void fill(Color color) override;
  void clear() override;

 protected:
  /** 2-bit grayscale pixel storage: 4 pixels/byte packed MSB-first. */
  void draw_pixel_at(int x, int y, Color color) override;

  /** Mode-specific init: sends standard init sequence, then grayscale LUT if needed. */
  bool initialise(bool partial) override;

  /**
   * Transfer data to display.
   * In grayscale mode, sends two passes (0x24 and 0x26 planes).
   * In mono mode, delegates to EPaperMono.
   */
  bool transfer_data() override;

  /** Refresh command varies by display mode. */
  void refresh_screen(bool partial) override;

  /** Buffer length as set by active display mode. */
  size_t get_active_buffer_length_() const {
    return (this->display_mode_ == SSD1683DisplayMode::GRAYSCALE4) ? this->gray_buffer_length_ : this->buffer_length_;
  }

  /** Send the grayscale LUT via the sequence of split commands expected by SSD1683.
   *  LUT_ALL[0..226]  → cmd 0x32 (waveform)
   *  LUT_ALL[227]     → cmd 0x3F (option)
   *  LUT_ALL[228]     → cmd 0x03 (gate level VGH)
   *  LUT_ALL[229..231]→ cmd 0x04 (source levels VSH1/VSH2/VSL)
   *  LUT_ALL[232]     → cmd 0x2C (VCOM)
   */
  void send_gray_lut_();

  /** Set the RAM window and cursor for a full-screen write. */
  void set_full_window_();

  /** Override set_window for SSD1683 which uses 2-byte X addressing. */
  void set_window() override;

  // Grayscale LUT supplied from Python at code generation time
  const uint8_t *gray_lut_{};
  size_t gray_lut_length_{};

  // Grayscale buffer geometry
  uint16_t gray_row_width_{};    ///< width in bytes for 2bpp (width/4 rounded up)
  size_t gray_buffer_length_{};  ///< total grayscale buffer size in bytes

  SSD1683DisplayMode display_mode_{SSD1683DisplayMode::FULL};

  // Transfer state for grayscale (which plane we're sending)
  bool gray_sending_new_data_{false};  ///< false = sending 0x24 (old), true = sending 0x26 (new)
};

}  // namespace esphome::epaper_spi
