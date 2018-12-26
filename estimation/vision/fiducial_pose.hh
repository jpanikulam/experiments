#pragma once
#include <opencv2/opencv.hpp>
//%deps(opencv)
#include <vector>
#include "sophus.hh"
#include <opencv2/aruco.hpp>
#include <assert.h>

namespace fiducials {
struct MarkerDetection{
    SE3 camera_to_marker_center;
    int id;
};
std::vector< MarkerDetection > detect_markers(cv::Mat mat);


class CalibrationManager{
    private:
    std::vector<cv::Mat> all_camera_images;

    public:
    void add_camera_image(cv::Mat image){
        all_camera_images.push_back(image);
    }

    int num_images_collected(){
        return all_camera_images.size();
    }

    void calibrate(){
        //based on https://github.com/sourishg/stereo-calibration/blob/master/calib_intrinsic.cpp
        assert(all_camera_images.size() > 0);
        int BOARD_WIDTH = 9;
        int BOARD_HEIGHT = 6;
        float SQUARE_SIZE = 1;
        auto board_size = cv::Size(BOARD_WIDTH, BOARD_HEIGHT);
        auto criteria = (cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.001);

        std::vector< std::vector< cv::Point3f > > object_points;
        std::vector< std::vector< cv::Point2f > > image_points;

        std::vector< cv::Point3f > obj; //simply a description of the board's intersection points
        for (int i = 0; i < BOARD_HEIGHT; i++)
          for (int j = 0; j < BOARD_WIDTH; j++)
            obj.push_back(cv::Point3f((float)j * SQUARE_SIZE, (float)i * SQUARE_SIZE, 0));



        for(auto const& image: all_camera_images) {
            cv::Mat gray;
            std::vector< cv::Point2f > corners;

            cv::cvtColor(image, gray, CV_BGR2GRAY);
            bool found = cv::findChessboardCorners(gray, board_size, corners,
                              CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
            if(found){
                cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                    cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
              image_points.push_back(corners);
              object_points.push_back(obj);
            }
        }

        cv::Mat K;
        cv::Mat D;
        std::vector< cv::Mat > rvecs, tvecs;
        int flag = 0;
        flag |= CV_CALIB_FIX_K4;
        flag |= CV_CALIB_FIX_K5;
        cv::calibrateCamera(object_points, image_points, all_camera_images[0].size(), K, D, rvecs, tvecs, flag);
        /*

        for fname in images:
            img = cv2.imread(fname)
            print img.shape
        #     break
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)
        #         cv2.imshow('img',img)
        #         cv2.waitKey(500)

        ret, mtx, distortion, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
        dist = distortion
        print distortion
        print mtx
        */
    }
};

}  // namespace fiducials