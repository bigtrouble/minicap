#ifndef MINICAP_PROJECTION_HPP
#define MINICAP_PROJECTION_HPP

#include <cmath>
#include <ostream>

class Projection {
public:
  class Parser {
  public:
    Parser(): mState(virtual_width_start) {
    }

    bool
    parse(Projection& proj, const char* lo, const char* hi) {
      consume: while (lo < hi) {
        switch (mState) {
        case virtual_width_start:
          if (isDigit(*lo)) {
            proj.virtualWidth += (*lo - 48);
            mState = virtual_width_continued;
            lo += 1;
            goto consume;
          }
          return false;
        case virtual_width_continued:
          if (isDigit(*lo)) {
            proj.virtualWidth *= 10;
            proj.virtualWidth += (*lo - 48);
            lo += 1;
            goto consume;
          }
          if (*lo == 'x') {
            mState = virtual_height_start;
            lo += 1;
            goto consume;
          }
          return false;
        case virtual_height_start:
          if (isDigit(*lo)) {
            proj.virtualHeight += (*lo - 48);
            mState = virtual_height_continued;
            lo += 1;
            goto consume;
          }
          return false;
        case virtual_height_continued:
          if (isDigit(*lo)) {
            proj.virtualHeight *= 10;
            proj.virtualHeight += (*lo - 48);
            lo += 1;
            goto consume;
          }
          return false;
        case satisfied:
          return false;
        }
      }

      return mState == virtual_height_continued;
    }

  private:
    enum State {
      virtual_width_start,
      virtual_width_continued,
      virtual_height_start,
      virtual_height_continued,
      satisfied,
    };

    State mState;

    inline bool
    isDigit(int input) {
      return input >= '0' && input <= '9';
    }
  };

  const uint32_t MAX_WIDTH = 10000;
  const uint32_t MAX_HEIGHT = 10000;

  uint32_t realWidth;
  uint32_t realHeight;
  uint32_t virtualWidth;
  uint32_t virtualHeight;
  uint32_t rotation;

  Projection()
    : realWidth(0),
      realHeight(0),
      virtualWidth(0),
      virtualHeight(0),
      rotation(0) {
  }

  void
  forceMaximumSize() {
    if (virtualWidth > realWidth) {
      virtualWidth = realWidth;
    }

    if (virtualHeight > realHeight) {
      virtualHeight = realHeight;
    }
  }

  void
  forceAspectRatio() {
    double aspect = static_cast<double>(realWidth) / static_cast<double>(realHeight);

    if (virtualHeight > (uint32_t) (virtualWidth / aspect)) {
      virtualHeight = static_cast<uint32_t>(round(virtualWidth / aspect));
    }
    else {
      virtualWidth = static_cast<uint32_t>(round(virtualHeight * aspect));
    }
  }

  bool
  valid() {
    return realWidth > 0 && realHeight > 0 &&
        virtualWidth > 0 && virtualHeight > 0 &&
        virtualWidth <= realWidth && virtualHeight <= realHeight;
  }

  friend std::ostream&
  operator<< (std::ostream& stream, const Projection& proj) {
    stream << proj.realWidth << 'x' << proj.realHeight << '@'
        << proj.virtualWidth << 'x' << proj.virtualHeight << '/' << proj.rotation;
    return stream;
  }
};

#endif
