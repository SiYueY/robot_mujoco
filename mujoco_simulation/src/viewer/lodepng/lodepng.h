#pragma once

#include <string>

enum LodePNGColorType {
  LCT_RGB = 2,
};

namespace lodepng {

unsigned encode(const std::string& filename, const unsigned char* image, unsigned width,
                unsigned height, LodePNGColorType colortype);

}  // namespace lodepng
