#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include "geometry/visualization/put_stl.hh"

#include "planning/jet/jet_dynamics.hh"
#include "planning/jet/jet_planner.hh"

#include "planning/jet/jet_model.hh"

// TODO CLEANUP
#include "viewer/primitives/camera.hh"
#include "viewer/primitives/image.hh"

#include "util/environment.hh"

#include "eigen.hh"
#include "sophus.hh"

namespace planning {
namespace jet {

constexpr bool SHOW_CAMERA = true;
constexpr bool SHOW_CAMERA_PROJECTION = true;
constexpr bool WRITE_IMAGES = false;
constexpr bool PRINT_STATE = false;
constexpr bool DRAW_VEHICLE = true;
constexpr bool TRACK_VEHICLE = false;
constexpr bool VISUALIZE_TRAJECTORY = true;

namespace {
void setup() {
  const auto view = viewer::get_window3d("Mr. Jet, jets");
  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)), Eigen::Vector3d::Zero()));
  view->set_continue_time_ms(10);

  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};
  background->add_plane({ground});
  background->flip();
}

void put_camera_projection(viewer::SimpleGeometry& geo, const viewer::Camera& cam) {
  const auto proj = cam.get_projection();

  using VPoint = viewer::ViewportPoint;
  const auto bottom_left_ray = proj.unproject(VPoint(jcc::Vec2(-1.0, -1.0)));
  const auto top_left_ray = proj.unproject(VPoint(jcc::Vec2(1.0, -1.0)));
  const auto top_right_ray = proj.unproject(VPoint(jcc::Vec2(1.0, 1.0)));
  const auto bottom_right_ray = proj.unproject(VPoint(jcc::Vec2(-1.0, 1.0)));

  const jcc::Vec4 red(1.0, 0.1, 0.1, 0.8);
  constexpr double length = 5.0;
  const jcc::Vec3 origin = bottom_right_ray(0.0);
  geo.add_line({origin, bottom_left_ray(length), red});
  geo.add_line({origin, top_left_ray(length), red});
  geo.add_line({origin, top_right_ray(length), red});
  geo.add_line({origin, bottom_right_ray(length), red});

  geo.add_line({bottom_left_ray(length), top_left_ray(length), red});
  geo.add_line({top_left_ray(length), top_right_ray(length), red});
  geo.add_line({top_right_ray(length), bottom_right_ray(length), red});
  geo.add_line({bottom_right_ray(length), bottom_left_ray(length), red});
}

}  // namespace

void go() {
  setup();
  const auto view = viewer::get_window3d("Mr. Jet, jets");

  const std::string jet_path = jcc::Environment::asset_path() + "jetcat_p160.stl";
  const auto put_jet = geometry::visualization::create_put_stl(jet_path);

  const auto jet_tree = view->add_primitive<viewer::SceneTree>();
  const auto jet_geo = view->add_primitive<viewer::SimpleGeometry>();
  const auto accum_geo = view->add_primitive<viewer::SimpleGeometry>();

  {
    const auto image_tree = view->add_primitive<viewer::SceneTree>();
    const std::string fiducial_path = jcc::Environment::asset_path() + "fiducial.jpg";
    const cv::Mat fiducial_tag = cv::imread(fiducial_path);
    const auto image = std::make_shared<viewer::Image>(fiducial_tag);
    // view->add_primitive(image);
    image_tree->add_primitive("root", SE3(), "fiducial_1", image);

    image_tree->add_primitive(
        "root", SE3(SO3::exp(jcc::Vec3(1.0, 0.0, 0.0)), jcc::Vec3(-1.0, 0.0, 0.0)),
        "fiducial_2", image);
  }

  const auto camera = std::make_shared<viewer::Camera>();
  view->add_camera(camera);

  const JetModel model;
  if constexpr (DRAW_VEHICLE) {
    model.insert(*jet_tree);
  }

  State jet;
  jet.x = jcc::Vec3(-2.0, -2.0, 3.0);
  jet.w = jcc::Vec3(0.3, -0.4, 1.0);

  jet.throttle_pct = 0.0;

  bool target_achieved = false;
  const jcc::Vec3 final_target(0.5, 0.5, 0.9);
  const jcc::Vec3 intermediate_target = final_target + jcc::Vec3(0.0, 0.0, 1.0);

  std::vector<Controls> prev_controls;
  for (int j = 0; j < 1000 && !view->should_close(); ++j) {
    const jcc::Vec3 prev = jet.x;

    //
    // Planning state
    //

    if ((jet.x - intermediate_target).norm() < 0.25) {
      target_achieved = true;
    }
    const jcc::Vec3 target = target_achieved ? final_target : intermediate_target;
    Desires desire;
    if (target_achieved) {
      desire.supp_v_weight = 600.0;
    }
    desire.target = target;

    //
    // Execute planning
    //

    const auto future_states = plan(jet, desire, prev_controls);
    jet = future_states[1].state;

    //
    // Print out first state
    //

    const auto ctrl = future_states[1].control;
    if constexpr (PRINT_STATE) {
      std::cout << "\tq     : " << ctrl.q.transpose() << std::endl;
      std::cout << "\tx     : " << jet.x.transpose() << std::endl;
      std::cout << "\tw     : " << jet.w.transpose() << std::endl;
      std::cout << "\tv     : " << jet.v.transpose() << std::endl;
      std::cout << "\tthrust: " << jet.throttle_pct << std::endl;
    }

    //
    // Visualize
    //

    const SE3 world_from_jet = SE3(jet.R_world_from_body, jet.x);
    const SE3 jet_from_camera =
        SE3(SO3::exp(jcc::Vec3(0.1, -0.1, 0.0)), jcc::Vec3(0.1, 0.05, 0.1));

    if constexpr (VISUALIZE_TRAJECTORY) {
      for (std::size_t k = 0; k < future_states.size(); ++k) {
        const auto& state = future_states.at(k).state;
        const SE3 world_from_state = SE3(state.R_world_from_body, state.x);
        const double scale =
            static_cast<double>(k) / static_cast<double>(future_states.size());
        jet_geo->add_axes({world_from_state, 1.0 - scale});

        if (k > 1) {
          jet_geo->add_line({future_states.at(k).state.x, future_states.at(k - 1).state.x,
                             jcc::Vec4(0.8, 0.8, 0.1, 0.8), 5.0});
        }
      }
      jet_geo->add_line(
          {world_from_jet.translation(),
           world_from_jet.translation() +
               (world_from_jet.so3() * jcc::Vec3::UnitZ() * jet.throttle_pct * 0.1),
           jcc::Vec4(0.1, 0.9, 0.1, 0.8), 9.0});
      accum_geo->add_line({prev, jet.x, jcc::Vec4(1.0, 0.7, 0.7, 0.7), 5.0});
    }

    if constexpr (TRACK_VEHICLE) {
      const SO3 world_from_target_rot = SO3::exp(jcc::Vec3::UnitX() * 3.1415 * 0.5);
      const SE3 world_from_target(world_from_target_rot, world_from_jet.translation());
      view->set_target_from_world(world_from_target.inverse());
    } else {
      // view->set_target_from_world(world_from_jet.inverse());
    }

    jet_tree->set_world_from_root(world_from_jet);
    camera->set_world_from_camera(world_from_jet * jet_from_camera);
    accum_geo->flush();

    if constexpr (SHOW_CAMERA_PROJECTION) {
      put_camera_projection(*jet_geo, *camera);
    }
    const cv::Mat image = camera->extract_image();
    if (SHOW_CAMERA) {
      cv::imshow("Localization Camera", image);
      cv::waitKey(1);
    }
    if (WRITE_IMAGES) {
      if (j == 0) {
        std::cout << "Projection: " << std::endl;
        std::cout << camera->get_projection().projection_mat() << std::endl;
      }
      if (j % 1 == 0) {
        std::cout << "-----" << j << std::endl;
        std::cout << camera->get_projection().modelview_mat() << std::endl;
        cv::imwrite("jet_image_" + std::to_string(j) + ".jpg", image);
      }
    }

    jet_geo->flip();

    view->spin_until_step();
  }

  cv::destroyAllWindows();
}

}  // namespace jet
}  // namespace planning

int main() {
  planning::jet::go();
}
