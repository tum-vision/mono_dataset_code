#include "markerdetector.h"
#include "posetracker.h"
#include "trackerimpl.h"


namespace aruco {

class ARUCO_EXPORT DFCMarkerTracker
{
public:
    DFCMarkerTracker();
    void setParams(const aruco::CameraParameters& camPar, const float mSize);
    void loadParamsFromFile(const std::string &path);
    void setDictionary(const std::string &dict, float error_corretion_rate= 0);

    MarkerDetector& getDetector(){return TheDetector;}
    std::map<int, cv::Ptr<TrackerImpl>> track(const cv::Mat& input, float thresh_confidence=0.f);
    std::vector<Dictionary> getDictionaries();
    void estimatePose();
    void draw(cv::Mat&img, cv::Scalar borderColor = cv::Scalar(0,0,255), cv::Scalar inforColor = cv::Scalar(255,0,255), bool trust=true);
    std::map<int, MarkerPoseTracker> getMapMultiPose(){return mapMultiPose;};

private:
    MarkerDetector TheDetector;
    std::map<int, cv::Ptr<TrackerImpl>> mapMultiTracker;
    std::map<int, MarkerPoseTracker> mapMultiPose;
    int frame=0;
    aruco::CameraParameters CamParam;
    float MarkerSize;

    //To config
    int arucoDetectRate = 10; //Rate to call aruco detect

    //Tracker params
    double psr=5.7; //threshold peak response
    int tau_s=32;   //filter size
};
}
