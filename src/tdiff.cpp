/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "ffmpeg_image_transport/tdiff.h"
#include <iomanip>

namespace ffmpeg_image_transport {
  std::ostream &operator<<(std::ostream &os, const TDiff &td) {
    os << std::fixed << std::setprecision(4) <<
      td.duration_ * (td.cnt_ > 0 ? 1.0 / (double)td.cnt_ : 0);
    return os;
  }
 
}  // namespace
