#include "../common/dji_linux_helpers.hpp"
#include <dji_vehicle.hpp>
#include <dji_advanced_sensing.hpp>
#include <dji_camera_image.hpp>
#include <iostream>
#include <pthread.h>
#include <thread>

#ifdef OPEN_CV_INSTALLED
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
using namespace cv;
#endif

using namespace DJI::OSDK;
using namespace std;





struct h264Img {
  uint8_t* __buf ;
  int __bufLen ;  
};


struct h264Img myH264Img;


void liveViewSampleCb(uint8_t* buf, int bufLen, void* userData) {

  cout << "liveViewSampleCb ..." << std::endl;
  myH264Img.__buf = buf;
  myH264Img.__bufLen = bufLen;

}


struct h264Img get_next_frame_h264(){
  return myH264Img;
}




class DJIDriver
{
public:
  DJIDriver() = delete;
  DJIDriver(int argc, char** argv)
    : _linuxEnvironment(argc, argv, true){

    };

  ~DJIDriver(){
    if (_cam_token == "FPV_CAM")
    {
      _vehicle->advancedSensing->stopFPVCameraStream();
    }
    if (_cam_token == "MAIN_CAM")
    {
      _vehicle->advancedSensing->stopMainCameraStream();
    }
  };

  bool setup(const std::string& cam_token){
    _vehicle = _linuxEnvironment.getVehicle();
    const char* acm_dev =
      _linuxEnvironment.getEnvironment()->getDeviceAcm().c_str();
    _vehicle->advancedSensing->setAcmDevicePath(acm_dev);
    _cam_token = cam_token;
    if (_vehicle == nullptr){
      std::cout << "Vehicle not initialized, exiting.\n";
      return false;
    }
    return true;
  };


  void startCapture(uint timeout_secs = 3){
    std::thread(&DJIDriver::startCapture_func, this).detach();
    std::this_thread::sleep_for(std::chrono::seconds(timeout_secs));
  }

  void stopCapture(uint timeout_secs = 6){
    _capture = false;
    std::this_thread::sleep_for(std::chrono::seconds(timeout_secs));
  }

  CameraRGBImage get_next_frame(){
    const std::lock_guard<std::mutex> lock(_mtx);
    return _current_img_rgb;
  };

private:
  static void prepare_image_cb(CameraRGBImage img, void* p){
    CameraRGBImage* obj = reinterpret_cast<CameraRGBImage*>(p);
    /// const std::lock_guard<std::mutex> lock(_mtx);
    // TODO LOCK
    cout << "#### Got image from:\t"
         << "name" << endl;
    *obj = std::move(img);
  };

  void startCapture_func(){
    _capture = true;
    if (_cam_token == "FPV_CAM")
    {
      auto camResult = _vehicle->advancedSensing->startFPVCameraStream(
        &prepare_image_cb, (void*)&this->_current_img_rgb);
    }
    else if (_cam_token == "MAIN_CAM")
    {
      auto camResult = _vehicle->advancedSensing->startMainCameraStream(
        &prepare_image_cb, (void*)&this->_current_img_rgb);
    }
    else if (_cam_token == "MAIN_CAM_H264_1")
    {

     auto camResult =  _vehicle->advancedSensing->startH264Stream(LiveView::OSDK_CAMERA_POSITION_NO_1, liveViewSampleCb, nullptr);
    
    }



    else
      std::runtime_error("Invalid Camera token [" + _cam_token + "].");
    while (_capture)
      std::this_thread::sleep_for(std::chrono::seconds(1));
  };

  LinuxSetup     _linuxEnvironment;
  Vehicle*       _vehicle;
  std::string    _cam_token;
  CameraRGBImage _current_img_rgb;
  std::mutex     _mtx;
  bool           _capture;
};



