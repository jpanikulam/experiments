#include "sophus.hh"

// TODO:
//  - Add cmake support for adding frames
//  - Document permitted groups
//  - Add conversion to generic transform so that it can be passed to functions

namespace frames {
enum class FrameId {
  CAMERA,   //
  BODY,     //
  VEHICLE,  //
  PILOT,    //
  LOCAL,    //
  WORLD     //
};

template <FrameId destination, FrameId source, typename GROUP = SE3>
class Transform {
 public:
  // Transform() {
  // }

  Transform(const GROUP& g) : destination_from_source_(g) {
  }

  template <FrameId other_source>
  Transform<destination, other_source, GROUP> operator*(const Transform<source, other_source, GROUP>& other) const {
    return destination_from_source_ * other.destination_from_source();
  }

  Transform<source, destination, GROUP> inverse() const {
    return {destination_from_source_.inverse()};
  }

  const GROUP& destination_from_source() const {
    return destination_from_source_;
  }

 private:
  GROUP destination_from_source_;
};
}