#include "adas/text_renderer.hpp"

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <ft2build.h>
#include FT_FREETYPE_H

namespace adas {
namespace ui {
namespace {

struct FontState {
  struct Glyph {
    int width = 0;
    int rows = 0;
    int left = 0;
    int top = 0;
    int advance = 0;
    std::vector<unsigned char> alpha;
  };
  FT_Library library = nullptr;
  FT_Face face = nullptr;
  std::unordered_map<unsigned long long, Glyph> cache;
  ~FontState() {
    if (face) FT_Done_Face(face);
    if (library) FT_Done_FreeType(library);
  }
};

std::unique_ptr<FontState> state;

const FontState::Glyph* cached_glyph(unsigned long code, int pixel_height,
                                     bool render) {
  if (!state || !state->face) return nullptr;
  const unsigned long long key =
      (static_cast<unsigned long long>(pixel_height) << 32) | code;
  const auto found = state->cache.find(key);
  if (found != state->cache.end()) return &found->second;
  FT_Set_Pixel_Sizes(state->face, 0, std::max(8, pixel_height));
  if (FT_Load_Char(state->face, code, render ? FT_LOAD_RENDER : FT_LOAD_DEFAULT) != 0)
    return nullptr;
  if (!render && FT_Render_Glyph(state->face->glyph, FT_RENDER_MODE_NORMAL) != 0)
    return nullptr;
  const FT_GlyphSlot slot = state->face->glyph;
  FontState::Glyph glyph;
  glyph.width = static_cast<int>(slot->bitmap.width);
  glyph.rows = static_cast<int>(slot->bitmap.rows);
  glyph.left = slot->bitmap_left;
  glyph.top = slot->bitmap_top;
  glyph.advance = static_cast<int>(slot->advance.x >> 6);
  glyph.alpha.resize(glyph.width * glyph.rows);
  for (int row = 0; row < glyph.rows; ++row) {
    const unsigned char* source = slot->bitmap.buffer + row * slot->bitmap.pitch;
    std::copy(source, source + glyph.width, glyph.alpha.begin() + row * glyph.width);
  }
  const auto inserted = state->cache.emplace(key, std::move(glyph));
  return &inserted.first->second;
}

std::vector<unsigned long> decode_utf8(const std::string& text) {
  std::vector<unsigned long> output;
  for (size_t i = 0; i < text.size();) {
    const unsigned char first = static_cast<unsigned char>(text[i]);
    unsigned long code = first;
    size_t count = 1;
    if ((first & 0xe0) == 0xc0 && i + 1 < text.size()) {
      code = ((first & 0x1f) << 6) |
             (static_cast<unsigned char>(text[i + 1]) & 0x3f);
      count = 2;
    } else if ((first & 0xf0) == 0xe0 && i + 2 < text.size()) {
      code = ((first & 0x0f) << 12) |
             ((static_cast<unsigned char>(text[i + 1]) & 0x3f) << 6) |
             (static_cast<unsigned char>(text[i + 2]) & 0x3f);
      count = 3;
    } else if ((first & 0xf8) == 0xf0 && i + 3 < text.size()) {
      code = ((first & 0x07) << 18) |
             ((static_cast<unsigned char>(text[i + 1]) & 0x3f) << 12) |
             ((static_cast<unsigned char>(text[i + 2]) & 0x3f) << 6) |
             (static_cast<unsigned char>(text[i + 3]) & 0x3f);
      count = 4;
    }
    output.push_back(code);
    i += count;
  }
  return output;
}

}  // namespace

bool initialize_text(const std::string& font_path) {
  std::unique_ptr<FontState> next(new FontState());
  if (FT_Init_FreeType(&next->library) != 0) return false;
  if (FT_New_Face(next->library, font_path.c_str(), 0, &next->face) != 0) return false;
  state = std::move(next);
  return true;
}

bool text_ready() { return state && state->face; }

cv::Size measure_text(const std::string& text, int pixel_height) {
  if (!text_ready()) {
    int baseline = 0;
    return cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX,
                           pixel_height / 28.0, 1, &baseline);
  }
  int width = 0;
  for (unsigned long code : decode_utf8(text)) {
    const FontState::Glyph* glyph = cached_glyph(code, pixel_height, false);
    if (glyph) width += glyph->advance;
  }
  return cv::Size(width, pixel_height);
}

void draw_text(cv::Mat& image, const std::string& text, const cv::Point& baseline,
               int pixel_height, const cv::Scalar& color) {
  if (!text_ready()) {
    cv::putText(image, text, baseline, cv::FONT_HERSHEY_SIMPLEX,
                pixel_height / 28.0, color, 1, cv::LINE_AA);
    return;
  }
  int pen_x = baseline.x;
  for (unsigned long code : decode_utf8(text)) {
    const FontState::Glyph* glyph = cached_glyph(code, pixel_height, true);
    if (!glyph) continue;
    const int left = pen_x + glyph->left;
    const int top = baseline.y - glyph->top;
    for (int row = 0; row < glyph->rows; ++row) {
      const int y = top + row;
      if (y < 0 || y >= image.rows) continue;
      const unsigned char* source = glyph->alpha.data() + row * glyph->width;
      for (int col = 0; col < glyph->width; ++col) {
        const int x = left + col;
        if (x < 0 || x >= image.cols) continue;
        const float alpha = source[col] / 255.0f;
        if (alpha <= 0.0f) continue;
        cv::Vec3b& pixel = image.at<cv::Vec3b>(y, x);
        for (int channel = 0; channel < 3; ++channel) {
          pixel[channel] = cv::saturate_cast<unsigned char>(
              pixel[channel] * (1.0f - alpha) + color[channel] * alpha);
        }
      }
    }
    pen_x += glyph->advance;
  }
}

}  // namespace ui
}  // namespace adas
