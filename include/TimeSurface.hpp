#ifndef TIME_SURFACE_HPP
#define TIME_SURFACE_HPP

#include <mutex>
#include <thread>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Time.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>

#include "Timer.hpp"
#include "CameraInfo.hpp"
#include "EventQueue.hpp"

namespace e_demo {

    /*********************
     * @class TimeSurface
     *********************/
    class TimeSurface {
    public:
        // ----- Enum -----
        // @brief  [TimeSurfaceMode]        
        enum class  TimeSurfaceMode {
            BACKWARD = 0x00,
            FORWARD = 0x01,
        };

        // ----- Constructor -----
        TimeSurface(ros::NodeHandle & nhPublic, ros::NodeHandle &nhPrivate) : nhPublic(nhPublic),  nhPrivate(nhPrivate), numTimeSurface(0) {
            // --- Init Sub ---
            subCameraInfo = nhPublic.subscribe("camera_info", 1, &TimeSurface::cameraInfoCallback, this);
            subSync = nhPublic.subscribe("sync", 1, &TimeSurface::syncCallback, this);
            subEvents = nhPublic.subscribe("events", 0, &TimeSurface::eventsCallback, this);
            // --- Init Pub ---
            image_transport::ImageTransport it(nhPublic);
            pubTimeSurface = it.advertise("time_surface", 1);
            // --- Init Param ---
            cameraInfo.initialized = isSensorInitialized = false;
            int mode;
            nhPrivate.param<int>("time_surface_mode", mode, 0);
            paramTimeSurfaceMode = (TimeSurfaceMode)mode;
            nhPrivate.param<bool>("output_fps", paramOutputFPS, false);
            nhPrivate.param<bool>("use_polarity", paramUsePolarity, false);
            nhPrivate.param<bool>("use_sim_time", paramUseSimTime, true);
            nhPrivate.param<int>("parallel_thread_number", paramParallelThreadNumber, 1);
            nhPrivate.param<int>("median_blur_kernel_size", paramMedianBlurKernelSize, 1);
            nhPrivate.param<int>("max_event_queue_length", paramMaxEventQueueLength, 100);
            nhPrivate.param<int>("max_event_queue_mat_length", paramMaxEventQueueMatLength, 20);
            nhPrivate.param<double>("decay_millisecond", paramDecayMillisecond, 30.0);            
        }

        // ----- Destructor -----
        virtual ~TimeSurface() {
            pubTimeSurface.shutdown();
        }
        
    private:
        // ----- Nh -----
        ros::NodeHandle nhPublic;
        ros::NodeHandle nhPrivate;
        // ----- Sub -----
        ros::Subscriber subCameraInfo;
        ros::Subscriber subSync;
        ros::Subscriber subEvents;
        // ----- Pub -----
        image_transport::Publisher pubTimeSurface;
        // ----- Data -----
        std::mutex dataMutex;
        long numTimeSurface;
        bool isSensorInitialized;
        cv::Size sensorSize;
        std::shared_ptr<EventQueue> pEventQueue;
        std::shared_ptr<EventQueueMat> pEventQueueMat;
        // ----- Param -----
        TimeSurfaceMode paramTimeSurfaceMode;
        bool paramOutputFPS;
        bool paramUsePolarity;
        bool paramUseSimTime;
        int paramParallelThreadNumber;
        int paramMedianBlurKernelSize;
        int paramMaxEventQueueLength;
        int paramMaxEventQueueMatLength;
        double paramDecayMillisecond;
        // ----- Timer -----
        Timer timer;
        // ----- Camera -----
        CameraInfo cameraInfo;
        // ----- Core -----
        // @brief  [create] create time surface at sync time
        void create(const ros::Time& syncTime) {
            std::lock_guard<std::mutex> locker(dataMutex);
            if (!cameraInfo.initialized || !isSensorInitialized)
                return;
            // --- Create ---
            cv::Mat timeSurfaceMap = cv::Mat::zeros(sensorSize, CV_64F);
            if (paramParallelThreadNumber > 1) {
                // Multi-thread
                int threadNum = std::min(paramParallelThreadNumber,  sensorSize.width);
                std::vector<std::thread> threadList;
                int d = sensorSize.width / threadNum;
                for (int i = 0; i < threadNum - 1; i++) {
                    threadList.emplace_back(std::thread(std::bind(&TimeSurface::calculate, this, pEventQueueMat, syncTime, 
                        0, sensorSize.height - 1, d * i, d * i + d - 1, timeSurfaceMap)));
                }
                threadList.emplace_back(std::thread(std::bind(&TimeSurface::calculate, this, pEventQueueMat, syncTime, 
                        0, sensorSize.height - 1, d * threadNum - d, sensorSize.width - 1, timeSurfaceMap)));
                for(std::thread &thread : threadList)
                    if(thread.joinable())
                        thread.join();
            }
            else {
                // Single-thread
                calculate(pEventQueueMat, syncTime, 0, sensorSize.height - 1, 0, sensorSize.width - 1, timeSurfaceMap);    
            }
            // --- Process ---
            if (paramUsePolarity)
                timeSurfaceMap= 255.0 * ((timeSurfaceMap + 1.0) / 2.0);     // [-1, 1] -> [0, 255]
            else
                timeSurfaceMap = 255.0 * timeSurfaceMap;    // [0, 1] -> [0, 255]
            timeSurfaceMap.convertTo(timeSurfaceMap, CV_8U);   // double(64)-> unsigned int(8)
            if (paramMedianBlurKernelSize > 0)
                cv::medianBlur(timeSurfaceMap, timeSurfaceMap, 2 * paramMedianBlurKernelSize + 1);
            // ---  Publish ---
            if (pubTimeSurface.getNumSubscribers() > 0) {
                static cv_bridge::CvImage cvImage;
                cvImage.encoding = "mono8";
                cvImage.image = timeSurfaceMap.clone();
                switch (paramTimeSurfaceMode) {
                    case TimeSurfaceMode::BACKWARD: {
                        cv_bridge::CvImage cvImageUndistort;
                        cvImageUndistort.encoding = cvImage.encoding;
                        cv::remap(cvImage.image, cvImageUndistort.image, cameraInfo.undistortMap1, cameraInfo.undistortMap1, CV_INTER_LINEAR);
                        cvImageUndistort.header.stamp = syncTime;
                        pubTimeSurface.publish(cvImageUndistort.toImageMsg());
                        break;
                    }
                    case TimeSurfaceMode::FORWARD: {
                        cvImage.header.stamp = syncTime;
                        pubTimeSurface.publish(cvImage.toImageMsg());
                        break;
                    }
                    default: break;
                }
            }   // publish end
            numTimeSurface++;
            // --- Time ---
            if (paramOutputFPS) {
                timer.toc();
                if (numTimeSurface % 100 == 0) {
                    ROS_INFO("FPS: (%d)", timer.fps());
                    timer.tic();
                }
            }
        }

        void calculate(const std::shared_ptr<EventQueueMat> pEventQueueMat, const ros::Time& syncTime, 
            const int rowBegin, const int rowEnd, const int colBegin, const int colEnd,  cv::Mat &timeSurfaceMap) {
            for(int y=rowBegin; y <= rowEnd; y++) {     // row
                for(int x=colBegin; x <= colEnd; x++) {     //  col
                        // Search Most Recent Event
                        const dvs_msgs::Event *mostRecentEvent = pEventQueueMat->recent(x , y, syncTime); // (x, y) && ts < syncTime
                        if (!mostRecentEvent)
                            continue;
                        const ros::Time& mostRecentEventTime = mostRecentEvent->ts;
                        if (mostRecentEventTime.toSec() <= 0)
                            continue;
                        // Calculate Exponential Value
                        const double dtVal = (syncTime - mostRecentEventTime).toSec();
                        double expVal = std::exp(-dtVal / (paramDecayMillisecond / 1000.0));
                        if (paramUsePolarity)
                            expVal *= (mostRecentEvent->polarity ? 1.0 : -1.0);
                        // Update Time Surface
                        switch (paramTimeSurfaceMode) {
                            case TimeSurfaceMode::BACKWARD: {
                                timeSurfaceMap.at<double>(y, x) = expVal;
                                break;
                            }
                            case TimeSurfaceMode::FORWARD: {
                                cv::Point2f undistortPoint =  cameraInfo.undistort(y, x);
                                int u = static_cast<int>(std::floor(undistortPoint.x));     // col
                                int v = static_cast<int>(std::floor(undistortPoint.y));     // row
                                if (u >= 0 && v >= 0 && u + 1 <  sensorSize.width && v + 1 < sensorSize.height) {
                                    double du = undistortPoint.x - u;
                                    double dv = undistortPoint.y - v;
                                    timeSurfaceMap.at<double>(v, u) += expVal * (1.0 - du) * (1.0 - dv);
                                    timeSurfaceMap.at<double>(v, u + 1) += expVal * du * (1.0 - dv);
                                    timeSurfaceMap.at<double>(v + 1, u) += expVal * (1.0 - du) * dv;
                                    timeSurfaceMap.at<double>(v + 1, u + 1) += expVal * du * dv;
                                    timeSurfaceMap.at<double>(v, u) = std::max(timeSurfaceMap.at<double>(v, u), 1.0);
                                    timeSurfaceMap.at<double>(v, u + 1) = std::max(timeSurfaceMap.at<double>(v, u + 1), 1.0);
                                    timeSurfaceMap.at<double>(v + 1, u) = std::max(timeSurfaceMap.at<double>(v + 1, u), 1.0);
                                    timeSurfaceMap.at<double>(v + 1, u + 1) = std::max(timeSurfaceMap.at<double>(v + 1, u + 1), 1.0);
                                }
                                break;
                            }
                            default: break;
                        }
                }
            }   // loop end
        }

        // ----- Callback -----
        // @brief  [cameraInfoCallback] topic - "camera_info"
        void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
            // --- Init CameraInfo ---
            if (cameraInfo.initialized)
                return;
            cameraInfo = CameraInfo(msg);
        }

        // @brief  [syncCallback] topic - "sync"
        void syncCallback(const std_msgs::TimeConstPtr& msg) {
            // --- Create Time Surface ---
             if (!cameraInfo.initialized || !isSensorInitialized)
                return;
            ros::Time syncTime = paramUseSimTime ?  ros::Time::now() :  msg->data;
            create(syncTime);
        }

        // @brief  [eventsCallback] topic - "events"
        void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg) {
            std::lock_guard<std::mutex> locker(dataMutex);
            // --- Init SensorSize ---
            if (!isSensorInitialized) {
                sensorSize = cv::Size(msg->width, msg->height);
                pEventQueue.reset(new EventQueue(paramMaxEventQueueLength));
                pEventQueueMat.reset(new EventQueueMat(msg->width, msg->height, paramMaxEventQueueMatLength));
                ROS_INFO("Sensor size: (%d x %d)", sensorSize.width, sensorSize.height);
                isSensorInitialized = true;
            }
            // --- Insert Data ---
            for(const dvs_msgs::Event& e : msg->events) {
                pEventQueue->insert(e);
                pEventQueueMat->insert(e);
            }
        };
    };

}

#endif
