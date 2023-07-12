#include "markerdetector.h"
#include "posetracker.h"
#include "trackerimpl.h"


namespace aruco {

class ARUCO_EXPORT DFCMarkerMapTracker : public MarkerMapPoseTracker
{
public:
    DFCMarkerMapTracker();
    void loadParamsFromFile(const std::string &path);
    void setDictionary(const std::string &dict, float error_corretion_rate= 0);
    std::vector<Dictionary> getDictionaries();
    MarkerDetector& getDetector(){return TheDetector;}
    void track(const cv::Mat& input);
    bool estimatePose();
    void drawMarkers(cv::Mat& img, float alpha=0.8, float trust=true);
    bool prediction(std::vector<cv::Point3f>&p3d, std::vector<cv::Point2f>&pd2);


private:
    MarkerDetector TheDetector;
    std::map<int, cv::Ptr<TrackerImpl>> mapMultiTracker;
    int frame=0;
    cv::Mat grey;
    double hull_factor;

    //To config
    int arucoDetectRate = 10; //Rate to call aruco detect
    float tau_c = 0.1; //Threshold hull_factor (tau_c paper)
    //Tracker params
    double psr=5.7; //threshold peak response
    int tau_s=32;   //filter size
};
}
