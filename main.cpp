#include "Camera.h"
#include "detector.h"
#include <opencv2/core.hpp>

int main() {
  HDURM::HKcam hkcam;
  hkcam.OpenCam("2BDFA2166410");
  hkcam.SetParam();
  std::unique_ptr<rm_auto_aim::Detector> autoAim;
  VisionSendData data;
  cv::Mat img;
  while (1) {
    hkcam.GetFlame(img);
    autoAim->run(img, 0, data);
    cv::waitKey(1);
  }
  hkcam.CloseCam();
  return 0;
}