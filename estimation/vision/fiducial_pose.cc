#include "estimation/vision/fiducial_pose.hh"
#include "eigen.hh"
//%deps(opencv)

namespace fiducials {

std::vector<MarkerDetection> detect_markers(cv::Mat inputImage) {
  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f> > corners;
  cv::aruco::detectMarkers(inputImage, dictionary, corners, ids);
  // cv::aruco::drawDetectedMarkers(inputImage, corners, ids);
  std::vector<cv::Vec3d> rvecs, tvecs;

// K[534.7753963605861, 0, 295.6532677968685;
//  0, 319.4378144747052, 353.0906557863973;
//  0, 0, 1]D[17.20290777181445, -36.81734552582041, 0.4520738079453668, 1.280789563889454, -637.6445013265748]

  // cv::Mat cameraMatrix =
  //     (cv::Mat1d(3, 3) << 230.8285372725757, 0, 348.18055411014,
  //                        0, 256.8163918879517, 386.7013296170525,
  //                        0, 0, 1);;

  cv::Mat cameraMatrix =
      (cv::Mat1d(3, 3) <<    320,  0, 320,
                             0, 320, 320,
                             0,    0,   1);

// cv::Mat cameraMatrix =
//       (cv::Mat1d(3, 3) << 407.1532, 0, 302.80,
//                              0, 395.88, 308.25,
//                              0, 0, 1);
      
  // webcam
  // cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 509.55744588, 0, 331.84201483,
  //                                            0, 512.03515271, 250.87030742,
  //                                            0, 0, 1);
  // cv::Mat distortionCoefficients = (cv::Mat1d(1, 8) << -0.0953, 0.108, 0.0455, -0.0312, -0.032);
  cv::Mat distortionCoefficients = (cv::Mat1d(1, 8) << 0,0,0,0,0);

  cv::aruco::estimatePoseSingleMarkers(corners, 0.49375, cameraMatrix,
                                       distortionCoefficients, rvecs, tvecs);
  // The returned transformation is the one that transforms points from each marker 
  // coordinate system to the camera coordinate system. The marker corrdinate system 
  // is centered on the middle of the marker, with the Z axis perpendicular to the marker plane. 
  std::vector<MarkerDetection> detections;
  // draw axis for each marker
  for (int i = 0; i < ids.size(); i++) {
    cv::aruco::drawAxis(inputImage, cameraMatrix, distortionCoefficients,
                        rvecs[i], tvecs[i], 1);
    auto camera_from_marker_center = SE3(SO3::exp(jcc::Vec3(rvecs[i][0], rvecs[i][1], rvecs[i][2])),
                                                jcc::Vec3(tvecs[i][0], tvecs[i][1], tvecs[i][2]));

    std::cout << "raw tvec "<< tvecs[i][0] << " " << tvecs[i][1] << " " << tvecs[i][2] << " " << std::endl;
    MarkerDetection detection;
    detection.marker_center_from_camera = camera_from_marker_center.inverse();
    detection.id = ids[i];
    detections.push_back(detection);
  }

  return detections;
}
}  // namespace fiducials