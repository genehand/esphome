#include "epaper_ssd1683.h"

#include "esphome/core/log.h"

namespace esphome::epaper_spi {

static constexpr const char *const TAG = "epaper_spi.ssd1683";

// ----------------------------------------------------------------------------
// Grayscale color mapping
// ----------------------------------------------------------------------------

/**
 * Map ESPHome Color to a 2-bit gray level.
 *
 * Gray levels (0–3):
 *   0 = black
 *   1 = dark gray
 *   2 = light gray
 *   3 = white
 *
 * Luminance is computed as sum of R + G + B, range 0–765.
 * Thresholds split the range into quarters.
 */
static uint8_t color_to_gray4(Color color) {
  int lum = static_cast<int>(color.r) + color.g + color.b;
  if (lum < 192)
    return 0;  // black
  if (lum < 384)
    return 2;  // dark gray (maps to gray2 = 0b10)
  if (lum < 576)
    return 1;  // light gray (maps to gray1 = 0b01)
  return 3;    // white
}

// ----------------------------------------------------------------------------
// Buffer operations
// ----------------------------------------------------------------------------

/**
 * Draw a single pixel into the 2bpp grayscale buffer.
 * Pixels are packed 4 per byte, MSB-first (pixel 0 in bits 7:6).
 */
void HOT EPaperSSD1683::draw_pixel_at(int x, int y, Color color) {
  if (this->display_mode_ != SSD1683DisplayMode::GRAYSCALE4) {
    // Fall back to monochrome pixel drawing from EPaperMono/EPaperBase
    EPaperMono::draw_pixel_at(x, y, color);
    return;
  }
  if (!this->rotate_coordinates_(x, y))
    return;

  const uint8_t gray = color_to_gray4(color);
  // 4 pixels per byte, 2 bits each; pixel 0 is at bits 7:6
  const size_t byte_pos = (y * this->gray_row_width_) + x / 4;
  const uint8_t shift = 6 - ((x % 4) * 2);
  const auto orig = this->buffer_[byte_pos];
  this->buffer_[byte_pos] = (orig & ~(0x03 << shift)) | (gray << shift);
}

void EPaperSSD1683::fill(Color color) {
  if (this->display_mode_ != SSD1683DisplayMode::GRAYSCALE4) {
    EPaperMono::fill(color);
    return;
  }
  if (this->get_clipping().is_set()) {
    EPaperBase::fill(color);
    return;
  }
  const uint8_t g = color_to_gray4(color);
  const uint8_t byte_val = g | (g << 2) | (g << 4) | (g << 6);
  this->buffer_.fill(byte_val);
  this->x_low_ = 0;
  this->y_low_ = 0;
  this->x_high_ = this->width_;
  this->y_high_ = this->height_;
}

void EPaperSSD1683::clear() {
  if (this->display_mode_ != SSD1683DisplayMode::GRAYSCALE4) {
    EPaperMono::clear();
    return;
  }
  this->fill(Color::WHITE);
}

// ----------------------------------------------------------------------------
// LUT and window helpers
// ----------------------------------------------------------------------------

/**
 * Send the 233-byte grayscale LUT to the display using the SSD1683
 * split-command protocol.
 *
 * Layout (from Waveshare reference):
 *   [0..226]   → 0x32  waveform LUT (227 bytes)
 *   [227]      → 0x3F  LUT option
 *   [228]      → 0x03  gate voltage VGH
 *   [229..231] → 0x04  source voltages VSH1, VSH2, VSL
 *   [232]      → 0x2C  VCOM level
 */
void EPaperSSD1683::send_gray_lut_() {
  if (this->gray_lut_ == nullptr || this->gray_lut_length_ < 233) {
    ESP_LOGE(TAG, "Gray LUT is missing or too short");
    return;
  }
  this->cmd_data(0x32, this->gray_lut_, 227);
  this->cmd_data(0x3F, {this->gray_lut_[227]});
  this->cmd_data(0x03, {this->gray_lut_[228]});
  this->cmd_data(0x04, {this->gray_lut_[229], this->gray_lut_[230], this->gray_lut_[231]});
  this->cmd_data(0x2C, {this->gray_lut_[232]});
}

void EPaperSSD1683::set_full_window_() {
  // Set RAM X window: bytes 0..(width/8-1), cursor at 0
  const uint8_t x_end = (this->width_ - 1) >> 3;
  this->cmd_data(0x44, {0x00, x_end});
  this->cmd_data(0x4E, {0x00});

  // Set RAM Y window: rows 0..(height-1), cursor at 0
  const uint8_t y_end_lo = (this->height_ - 1) & 0xFF;
  const uint8_t y_end_hi = (this->height_ - 1) >> 8;
  this->cmd_data(0x45, {0x00, 0x00, y_end_lo, y_end_hi});
  this->cmd_data(0x4F, {0x00, 0x00});
}

void EPaperSSD1683::set_window() {
  // SSD1683: RAM X address uses 5 bits (0-31), so only 2 bytes needed (start, end)
  // round x-coordinates to byte boundaries
  this->x_low_ &= ~7;
  this->x_high_ += 7;
  this->x_high_ &= ~7;
  this->cmd_data(0x44, {(uint8_t) (this->x_low_ / 8), (uint8_t) ((this->x_high_ - 1) / 8)});
  this->cmd_data(0x4E, {(uint8_t) (this->x_low_ / 8)});
  // RAM Y address uses 9 bits (0-511), so 4 bytes needed (start_lo, start_hi, end_lo, end_hi)
  this->cmd_data(0x45, {(uint8_t) this->y_low_, (uint8_t) (this->y_low_ / 256), (uint8_t) (this->y_high_ - 1),
                        (uint8_t) ((this->y_high_ - 1) / 256)});
  this->cmd_data(0x4F, {(uint8_t) this->y_low_, (uint8_t) (this->y_low_ / 256)});
}

// ----------------------------------------------------------------------------
// State machine overrides
// ----------------------------------------------------------------------------

void EPaperSSD1683::setup() {
  // set_display_mode() is called by codegen before setup(), so the mode is known here.
  // Promote buffer_length_ to grayscale size so EPaperBase::setup() allocates enough RAM.
  if (this->display_mode_ == SSD1683DisplayMode::GRAYSCALE4) {
    this->buffer_length_ = this->gray_buffer_length_;
  }
  EPaperBase::setup();
}

bool EPaperSSD1683::initialise(bool partial) {
  if (this->display_mode_ != SSD1683DisplayMode::GRAYSCALE4) {
    // Mono mode: use the standard init from EPaperMono (sends init_sequence_)
    // then set mono window using EPaperWaveshare/EPaperMono set_window
    // (set_window is called in transfer_data, nothing extra needed here)
    if (!EPaperMono::initialise(partial)) {
      return false;
    }
    this->current_update_is_partial_ = partial && this->display_mode_ == SSD1683DisplayMode::PARTIAL;
    if (this->current_update_is_partial_) {
      // Partial mode setup per Waveshare example
      this->cmd_data(0x21, {0x00, 0x00});  // Display update control (no bypass)
      this->cmd_data(0x3C, {0x80});        // Border waveform for partial
    } else {
      // Full and Fast modes: set display update control with bypass OTP
      this->cmd_data(0x21, {0x40, 0x00});  // Display update control: bypass OTP
      if (this->display_mode_ == SSD1683DisplayMode::FAST) {
        // For fast mode, set temperature register and load fast LUT
        // 0x6E = ~1.5s refresh, 0x5A = ~1s refresh
        this->cmd_data(0x1A, {0x6E});  // Write to temperature register
        // Dummy update cycle to load fast mode LUT
        this->cmd_data(0x22, {0x91});  // Enable Clock, Load LUT Mode 1, Disable Clock
        this->command(0x20);           // Master Activation
      }
    }
    return true;
  }

  // Grayscale init adapted from EPD_4IN2_V2_Init_4Gray()
  // Note: reset() already sent 0x12 (SWRESET) and waited for idle

  // Display update control for grayscale mode
  // Note: For grayscale, we do NOT set the bypass OTP bit (0x40)
  // The LUT commands (0x32, 0x3F, etc.) directly configure the waveform
  this->cmd_data(0x21, {0x00, 0x00});

  // Border waveform: GS Transition = 0 (VSS), VBD = 0
  this->cmd_data(0x3C, {0x03});

  // Booster soft-start control
  this->cmd_data(0x0C, {0x8B, 0x9C, 0xA4, 0x0F});

  // Load the grayscale LUT
  this->send_gray_lut_();

  // Data entry mode: increment X then Y
  this->cmd_data(0x11, {0x03});

  // Set full display window and cursor
  this->set_full_window_();

  this->send_red_ = false;  // we manage both planes ourselves
  return true;
}

bool HOT EPaperSSD1683::transfer_data_fast_() {
  // Fast mode: write same buffer data to both BW (0x24) and RED (0x26) planes
  auto start_time = millis();

  if (this->current_data_index_ == 0 && !this->fast_sending_red_) {
    // First pass: set window and start BW plane (0x24)
    this->set_window();
    this->command(0x24);
  }

  size_t row_length = (this->x_high_ - this->x_low_) / 8;
  this->start_data_();
  while (this->current_data_index_ != this->y_high_) {
    size_t data_idx = this->current_data_index_ * this->row_width_ + this->x_low_ / 8;
    for (size_t i = 0; i != row_length; i++) {
      this->write_byte(this->buffer_[data_idx++]);
    }
    ++this->current_data_index_;
    if (millis() - start_time > MAX_TRANSFER_TIME) {
      this->disable();
      return false;
    }
  }
  this->disable();
  this->current_data_index_ = 0;

  if (!this->fast_sending_red_) {
    // First pass done, start second pass for RED plane (0x26)
    this->set_window();
    this->command(0x26);
    this->fast_sending_red_ = true;
    return false;  // come back for second plane
  }

  // Both passes done
  this->fast_sending_red_ = false;
  return true;
}

bool HOT EPaperSSD1683::transfer_data_partial_() {
  // Partial mode: write only to BW plane (0x24)
  // The controller compares old vs new BW data for partial refresh
  auto start_time = millis();

  if (this->current_data_index_ == 0) {
    // Set window and start BW plane (0x24)
    this->set_window();
    this->command(0x24);
  }

  size_t row_length = (this->x_high_ - this->x_low_) / 8;
  this->start_data_();
  while (this->current_data_index_ != this->y_high_) {
    size_t data_idx = this->current_data_index_ * this->row_width_ + this->x_low_ / 8;
    for (size_t i = 0; i != row_length; i++) {
      this->write_byte(this->buffer_[data_idx++]);
    }
    ++this->current_data_index_;
    if (millis() - start_time > MAX_TRANSFER_TIME) {
      this->disable();
      return false;
    }
  }
  this->disable();
  this->current_data_index_ = 0;

  // Single pass for partial mode
  return true;
}

bool HOT EPaperSSD1683::transfer_data() {
  if (this->display_mode_ == SSD1683DisplayMode::FAST ||
      (this->display_mode_ == SSD1683DisplayMode::PARTIAL && !this->current_update_is_partial_)) {
    // Fast mode or first FULL update in PARTIAL mode: write same buffer to both planes
    return this->transfer_data_fast_();
  }
  if (this->current_update_is_partial_) {
    // Partial mode: write only to BW plane
    return this->transfer_data_partial_();
  }
  if (this->display_mode_ != SSD1683DisplayMode::GRAYSCALE4) {
    // Monochrome mode: delegate entirely to EPaperMono
    return EPaperMono::transfer_data();
  }

  // Grayscale mode: split 2bpp buffer into two 1bpp planes
  //
  // For each pixel with gray value g (0–3, stored as 2 bits):
  //   bit1 (MSB) of g → old data plane (0x24)
  //   bit0 (LSB) of g → new data plane (0x26)
  //
  // Waveshare LUT encoding:
  //   white (g=3=0b11): old=1, new=1
  //   gray2 (g=2=0b10): old=1, new=0
  //   gray1 (g=1=0b01): old=0, new=1
  //   black (g=0=0b00): old=0, new=0

  auto start_time = millis();

  // Only initialize when starting first plane (gray_sending_new_data_ == false)
  // The second plane is started at the end of this function after completing first plane
  if (this->current_data_index_ == 0 && !this->gray_sending_new_data_) {
    // First call: set window and start plane 0x24
    const uint8_t x_end = (this->width_ - 1) >> 3;
    const uint8_t y_end_lo = (this->height_ - 1) & 0xFF;
    const uint8_t y_end_hi = (this->height_ - 1) >> 8;
    this->cmd_data(0x44, {0x00, x_end});
    this->cmd_data(0x4E, {0x00});
    this->cmd_data(0x45, {0x00, 0x00, y_end_lo, y_end_hi});
    this->cmd_data(0x4F, {0x00, 0x00});

    this->command(0x24);
  }

  // The mono row_width_ (bytes per row in 1bpp) for this display
  const size_t mono_row_width = (this->width_ + 7) / 8;

  // We iterate over rows; current_data_index_ tracks current row
  this->start_data_();
  while (this->current_data_index_ < (size_t) this->height_) {
    // Extract one 1bpp row from the 2bpp buffer for the current plane
    for (size_t col_byte = 0; col_byte < mono_row_width; col_byte++) {
      uint8_t out = 0;
      // Each output byte covers 8 output pixels = 4 source bytes each holding 2 pixels
      for (int bit = 7; bit >= 0; bit--) {
        const size_t px = this->current_data_index_ * this->width_ + col_byte * 8 + (7 - bit);
        if (px >= (size_t) (this->width_ * this->height_)) {
          break;
        }
        const size_t src_byte = px / 4;
        const uint8_t src_shift = 6 - ((px % 4) * 2);
        const uint8_t gray = (this->buffer_[src_byte] >> src_shift) & 0x03;
        uint8_t plane_bit;
        if (!this->gray_sending_new_data_) {
          plane_bit = (gray >> 1) & 0x01;  // bit1 → old data (0x24)
        } else {
          plane_bit = gray & 0x01;  // bit0 → new data (0x26)
        }
        if (plane_bit)
          out |= (1 << bit);
      }
      this->write_byte(out);
    }
    this->current_data_index_++;

    if (millis() - start_time > MAX_TRANSFER_TIME) {
      this->disable();
      return false;
    }
  }
  this->disable();

  if (!this->gray_sending_new_data_) {
    // Finished plane 0x24; reset cursor and start plane 0x26
    const uint8_t x_end = (this->width_ - 1) >> 3;
    const uint8_t y_end_lo = (this->height_ - 1) & 0xFF;
    const uint8_t y_end_hi = (this->height_ - 1) >> 8;
    this->cmd_data(0x44, {0x00, x_end});
    this->cmd_data(0x4E, {0x00});
    this->cmd_data(0x45, {0x00, 0x00, y_end_lo, y_end_hi});
    this->cmd_data(0x4F, {0x00, 0x00});

    this->gray_sending_new_data_ = true;
    this->current_data_index_ = 0;
    this->command(0x26);
    return false;  // come back for the second plane
  }

  // Both planes done
  this->current_data_index_ = 0;
  this->gray_sending_new_data_ = false;
  return true;
}

void EPaperSSD1683::refresh_screen(bool partial) {
  if (this->display_mode_ == SSD1683DisplayMode::FAST) {
    this->cmd_data(0x22, {0xC7});
  } else if (this->display_mode_ == SSD1683DisplayMode::GRAYSCALE4) {
    this->cmd_data(0x22, {0xCF});
  } else if (this->current_update_is_partial_) {
    this->cmd_data(0x22, {0xFF});
  } else {
    this->cmd_data(0x22, {0xF7});
  }
  this->command(0x20);
}

}  // namespace esphome::epaper_spi
