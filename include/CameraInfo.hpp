#ifndef CAMERA_INFO_HPP
#define CAMERA_INFO_HPP

#include <sensor_msgs/CameraInfo.h>

namespace e_demo {
    /**********************
     * @struct CameraInfo
     * @ref http://docs.ros.org/en/kinetic/api/sensor_msgs/html/msg/CameraInfo.html
     **********************/
    struct CameraInfo {
        // ----- Parameter -----
        bool initialized;

        cv::Size size;
        cv::Mat cameraMatrix;                       // K(3×3)
        cv::Mat distortionCoefficients;      // D(1×5)
        cv::Mat rectificationMatrix;             // R(3×3)(stereo only)
        cv::Mat projectionMatrix;                // P(3×4)

        std::string distortionModel;
        cv::Mat undistortMap1, undistortMap2;            // for the whole image (cv::remap)
        cv::Mat_<cv::Point2f> undistortPointsList;      // for a specific point

        // ----- Function -----
        // @brief  [undistort] for the whole image
        // Call [cv::remap]

        // @brief  [undistort] for a specific point 
        // @param {int} y: row   x: col
        cv::Point2f undistort(const int y, const int x) {
            return undistortPointsList(y * size.width + x);
        }

        // ----- Constructor -----
        CameraInfo() { initialized = false; } 
        
        CameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg) {
            // --- Size ---
            size = cv::Size(msg->width, msg->height);
            // --- Camera Matrix (K) ---
            cameraMatrix = cv::Mat(3, 3, CV_64F);
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    cameraMatrix.at<double>(r, c) = msg->K[r * 3+ c];            
            // --- Distortion Coefficients (D) ---
            distortionCoefficients = cv::Mat(msg->D.size(), 1, CV_64F);
            for (int r = 0; r < static_cast<int>(msg->D.size()); r++)
                distortionCoefficients.at<double>(r) = msg->D[r];            
            // --- Rectification Matrix (R) ---
            rectificationMatrix = cv::Mat(3, 3, CV_64F);
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    rectificationMatrix.at<double>(r, c) = msg->R[r * 3+ c];
            // --- Projection Matrix (P) ---
            projectionMatrix = cv::Mat(3, 4, CV_64F);
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 4; c++)
                    projectionMatrix.at<double>(r, c) = msg->P[r * 4 + c];
            // --- Distortion Model ---
            distortionModel = msg->distortion_model;
            if (distortionModel == "plumb_bob") {
                cv::initUndistortRectifyMap(
                    cameraMatrix, distortionCoefficients, 
                    rectificationMatrix, projectionMatrix,
                    size, CV_32FC1, undistortMap1, undistortMap2
                );
                // Init undistortPointsList
                cv::Mat_<cv::Point2f> rawPointsList(1, size.width * size.height);
                undistortPointsList = cv::Mat_<cv::Point2f>(1, size.width * size.height);
                for (int y = 0; y < size.height; y++)   // row
                    for (int x = 0; x < size.width; x++)    // col
                        rawPointsList(y * size.width + x) = cv::Point2f((float) x, (float) y);
                cv::undistortPoints(
                    rawPointsList, undistortPointsList,
                    cameraMatrix, distortionCoefficients,
                    rectificationMatrix, projectionMatrix
                );
                initialized = true;
                ROS_INFO("CameraInfo::Camera information is loaded (Distortion model %s).", distortionModel.c_str());
            }
            else if (distortionModel == "equidistant") {
                //  Init undistortMap
                cv::fisheye::initUndistortRectifyMap(
                    cameraMatrix, distortionCoefficients, 
                    rectificationMatrix, projectionMatrix,
                    size, CV_32FC1, undistortMap1, undistortMap2
                );
                // Init undistortPointsList
                cv::Mat_<cv::Point2f> rawPointsList(1, size.width * size.height);
                undistortPointsList = cv::Mat_<cv::Point2f>(1, size.width * size.height);
                for (int y = 0; y < size.height; y++)   // row
                    for (int x = 0; x < size.width; x++)    // col
                        rawPointsList(y * size.width + x) = cv::Point2f((float) x, (float) y);
                cv::fisheye::undistortPoints(
                    rawPointsList, undistortPointsList, 
                    cameraMatrix, distortionCoefficients,
                    rectificationMatrix, projectionMatrix
                );
                initialized = true;
                ROS_INFO("CameraInfo::Camera information is loaded (Distortion model %s).", distortionModel.c_str());
            }
            else {
                initialized = false;
                ROS_ERROR_ONCE("CameraInfo::Distortion model %s is not supported.", distortionModel.c_str());
            }
        }
    };

}

#endif
