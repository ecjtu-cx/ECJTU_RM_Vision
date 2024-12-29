#include "Camera.h"

#include <opencv2/core.hpp>

int main() {
    HDURM::HKcam hkcam;
    hkcam.OpenCam("2BDFA2166410");
    hkcam.SetParam();
    cv::Mat img;
    while(1){
        hkcam.GetFlame(img);
        cv::imshow("img", img);
        cv::waitKey(1);
    }
    hkcam.CloseCam();
    return 0;
}