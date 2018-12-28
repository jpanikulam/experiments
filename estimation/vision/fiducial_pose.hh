#pragma once
#include <opencv2/opencv.hpp>
//%deps(opencv)
#include <vector>
#include "sophus.hh"
#include <opencv2/aruco.hpp>
#include <assert.h>

namespace estimation {
namespace vision {

struct MarkerDetection{
    SE3 marker_center_from_camera;
    int id;
};

struct MarkerInWorld{
    SE3 world_from_marker;
    int id;
};


std::vector< MarkerDetection > detect_markers(cv::Mat mat);

std::vector<MarkerInWorld> get_world_from_marker_centers(cv::Mat camera_image, SE3 world_from_opengl_camera);

class CalibrationManager{
    private:
    std::vector<cv::Mat> all_camera_images;

    public:
    void add_camera_image(cv::Mat image);

    int num_images_collected();

    std::pair<cv::Mat, cv::Mat> calibrate();
};

}  // namespace vision
}  // namespace estimation