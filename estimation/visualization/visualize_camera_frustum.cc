#include "estimation/visualization/visualize_camera_frustum.hh"

#include "logging/assert.hh"

namespace estimation {

void visualize_camera_frustum(viewer::SimpleGeometry& geo,
                              viewer::Ui2d& ui2d,
                              const NonlinearCameraModel& model) {
  const std::vector<jcc::Vec2> image_pts = {
      jcc::Vec2(0.0, model.rows()),           //
      jcc::Vec2(model.cols(), model.rows()),  //
  };

  constexpr double DIST_Z = 4.0;
  for (const auto& image_pt : image_pts) {
    const auto unproj = model.unproject(image_pt);
    JASSERT(static_cast<bool>(unproj), "Could not deproject a point");
    {
      geo.add_ray(*unproj, DIST_Z, jcc::Vec4::Ones());
      ui2d.add_point({image_pt / model.rows(), jcc::Vec4(1.0, 1.0, 0.2, 1.0), 5.0});
    }
  }

  {
    const jcc::Vec2 bottom_right(model.cols(), 0.0);
    const auto unproj = model.unproject(bottom_right);
    JASSERT(static_cast<bool>(unproj), "Could not deproject a point");
    {
      geo.add_ray(*unproj, DIST_Z, jcc::Vec4(1.0, 0.1, 0.1, 1.0));
      ui2d.add_point({bottom_right / model.rows(), jcc::Vec4(1.0, 0.0, 0.0, 1.0), 5.0});
    }
  }

  {
    const jcc::Vec2 bottom_left(0.0, 0.0);
    const auto unproj = model.unproject(bottom_left);
    JASSERT(static_cast<bool>(unproj), "Could not deproject a point");
    {
      geo.add_ray(*unproj, DIST_Z, jcc::Vec4(0.1, 1.0, 0.1, 1.0));
      ui2d.add_point({bottom_left / model.rows(), jcc::Vec4(0.0, 1.0, 0.0, 1.0), 5.0});
    }
  }
}
}  // namespace estimation