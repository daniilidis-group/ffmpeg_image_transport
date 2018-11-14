/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once
#include <iostream>

namespace ffmpeg_image_transport {

  class TDiff {
  public:
    friend std::ostream &operator<<(std::ostream &os, const TDiff &td);
    inline void update(double dt) {
      duration_ += dt;
      cnt_++;
    }
    inline void reset() {
      duration_ = 0;
      cnt_ = 0;
    }
  private:
    int64_t cnt_{0};
    double  duration_{0};
  };
  std::ostream &operator<<(std::ostream &os, const TDiff &td);

}

