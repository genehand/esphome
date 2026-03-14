#pragma once

#include "epaper_spi.h"
#include "epaper_spi_mono.h"

namespace esphome::epaper_spi {

enum class SSD1683DisplayMode : uint8_t {
  FULL = 0,        // Full refresh
  PARTIAL = 1,     // Partial update
  GRAYSCALE4 = 2,  // 4-shade grayscale
  FAST = 3,        // Fast refresh (may have ghosting)
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

  void setup() override;

  DisplayType get_display_type() override {
    if (this->display_mode_ == SSD1683DisplayMode::GRAYSCALE4)
      return DISPLAY_TYPE_GRAYSCALE;
    return DISPLAY_TYPE_BINARY;
  }

  void fill(Color color) override;
  void clear() override;

 protected:
  SSD1683DisplayMode display_mode_{SSD1683DisplayMode::FULL};

  const uint8_t *gray_lut_{};
  size_t gray_lut_length_{};
  uint16_t gray_row_width_{};             // width in bytes for 2bpp (width/4 rounded up)
  size_t gray_buffer_length_{};           // total grayscale buffer size in bytes

  bool gray_sending_new_data_{false};     // false = sending 0x24 (old), true = sending 0x26 (new)
  bool fast_sending_red_{false};          // false = sending 0x24 (BW), true = sending 0x26 (RED)
  bool partial_sending_red_{false};
  bool current_update_is_partial_{false};

  void draw_pixel_at(int x, int y, Color color) override;
  bool initialise(bool partial) override;
  bool transfer_data() override;
  void refresh_screen(bool partial) override;
  void send_gray_lut_();
  void set_full_window_();
  void set_window() override;
  bool transfer_data_fast_();
  bool transfer_data_partial_();

  size_t get_active_buffer_length_() const {
    return (this->display_mode_ == SSD1683DisplayMode::GRAYSCALE4) ? this->gray_buffer_length_ : this->buffer_length_;
  }
};

}  // namespace esphome::epaper_spi
