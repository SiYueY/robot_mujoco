#include "lodepng.h"

namespace lodepng {

unsigned encode(const std::string& filename, const unsigned char* image, unsigned width,
                unsigned height, LodePNGColorType colortype) {
  (void)filename;
  (void)image;
  (void)width;
  (void)height;
  (void)colortype;
  return 1U;
}

}  // namespace lodepng
