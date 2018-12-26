#include "estimation/vision/fiducial_pose.hh"
//%deps(opencv)

namespace fiducials {



std::vector< MarkerDetection > detect_markers(cv::Mat inputImage) {
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(inputImage, dictionary, corners, ids);
    // cv::aruco::drawDetectedMarkers(inputImage, corners, ids);
    std::vector< cv::Vec3d > rvecs, tvecs;
    cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 509.55744588, 0, 331.84201483,
                                               0, 512.03515271, 250.87030742,
                                               0, 0, 1);
    cv::Mat distortionCoefficients = (cv::Mat1d(1, 8) << 0, 0, 0, 0, 0, 0, 0, 0);

    cv::aruco::estimatePoseSingleMarkers(corners, 0.5, cameraMatrix, distortionCoefficients, rvecs, tvecs);

    std::vector< MarkerDetection > detections;
    // draw axis for each marker
    for (int i = 0; i<ids.size(); i++){
        cv::aruco::drawAxis(inputImage, cameraMatrix, distortionCoefficients, rvecs[i], tvecs[i], 0.1);
        auto marker_center_to_camera = SE3(SO3::exp(Eigen::Vector3d(0*rvecs[i][1], 0*rvecs[i][0], 0*rvecs[i][2])),
                              Eigen::Vector3d(-tvecs[i][0], -tvecs[i][1], tvecs[i][2]));
        MarkerDetection detection;
        detection.camera_to_marker_center = marker_center_to_camera.inverse();
        detection.id = ids[i];
        detections.push_back(detection);
    }


    return detections;
}
}  // namespace fiducials