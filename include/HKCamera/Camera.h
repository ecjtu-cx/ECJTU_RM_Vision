#ifndef HKCAM
#define HKCAM

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "MvCameraControl.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <iostream>

namespace HDURM
{
    class HKcam
    {

    public:
        /**
         * @brief
         *
         */
        HKcam();

        /**
         * @brief
         *
         * @return true
         * @return false
         */
        bool OpenCam(const std::string &cameraId = "");

        /**
         * @brief
         *
         * @return true
         * @return false
         */
        bool CloseCam();

        /**
         * @brief
         *
         */
        //        bool GetFlame(cv::Mat& img);
        bool GetFlame(cv::Mat &img);

        /**
         * @brief Set params for camera
         *
         */
        void SetParam();

        std::string GetCamName();

    private:
        // state num
        int nRet;

        // handle for manipulating the Camera
        void *handle;

        // camera param
        MVCC_INTVALUE stParam{};

        // frame ptr
        unsigned char *pData;

        // format of frame ,read from camera
        MV_FRAME_OUT_INFO_EX stImageInfo{};

        // indicating whether the camera is connected or not
        // a flag for daemon thread
        bool connected_flag;

        std::string id;
        int nImageOrientation;

    }; // HKcam

} // hnurm
#endif
