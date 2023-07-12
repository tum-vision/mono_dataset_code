/**
Copyright 2017 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
*/

#ifndef ArucoCalibrator_H
#define ArucoCalibrator_H
#include <string>
#include <vector>
#include "marker.h"
#include "markermap.h"
#include "cameraparameters.h"
#include <thread>
#include <mutex>
namespace  aruco {
/**
 * @brief The Calibrator class implements a calibration thread
 */
class Calibrator{

    std::vector<std::vector<aruco::Marker> >  vmarkers,vmarkerstmp;
    float _msize;
    aruco::MarkerMap _mmap;
    cv::Size _imageSize;
    std::thread _calibThread;
    std::mutex markersMutex,markerstmpMutex,infoMutex,calibrationMutex;
    std::string strinfo;
    bool keepCalibrating;
    aruco::CameraParameters cameraParams;
    float calibError=-1;
public:

    Calibrator();
    ~Calibrator();
    void setParams(cv::Size imageSize, float markerSize=1, std::string markerMap="");
    //add a detection of the calibration markerset and do calibration
    //returns the current number of views
    void addView(const std::vector<Marker> &markers);

    //returns a string with info about the status
    std::string getInfo();

    //obtain the camera calibration results. It is blocking function
    //returns true if calibration was ok
    bool getCalibrationResults( aruco::CameraParameters &camp);
    int getNumberOfViews();

    float getReprjError()const{return calibError;}
private:
    void calibration_function();
    void setInfoString(const std::string &str);
    void stopThread();
    void startThread();

   float  cameraCalibrate(std::vector<std::vector<aruco::Marker> >  &allMarkers, int imageWidth,int imageHeight,float markerSize,  aruco::MarkerMap &inmmap, aruco::CameraParameters&io);
    aruco::MarkerMap getDefaultCalibrationBoard();

};
}

#endif
