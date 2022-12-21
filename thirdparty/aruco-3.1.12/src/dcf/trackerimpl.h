// This is an adaptation of the Minimum Output Sum of Squared Error (MOSSE) tracking system.
// This approach allows to create correlation filters of the desired target with a high computer performance.
//
// D. S. Bolme, J. R. Beveridge, B. A. Draper and Y. M. Lui, "Visual object tracking using adaptive correlation filters,"
// 2010 IEEE Computer Society Conference on Computer Vision and Pattern Recognition, San Francisco, CA, 2010, pp. 2544-2550
//
// The system has been adapted for tracking of fiducial markers, where both the marker and its four corners are tracked
// In addition, the system works with different scale levels, adapting the filters according to the size of the marker.

#ifndef TrackerImpl_H
#define TrackerImpl_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "marker.h"
#include "map"
#include "timers.h"
#include "dictionary_based.h"

using namespace aruco;

class TrackerImpl
{
public:
    TrackerImpl(const Marker& marker, uint64_t code, uint32_t nbits, const cv::Mat& image, double psr=5.8, int tau_s=32);

    bool initImpl();
    bool updateImpl(const cv::Mat& imgPyr, float thres_confidence=0);
    cv::Rect2d getBoundingBox(){return boundingBox;}
    cv::Point2d getCenter(){return center;}
    std::vector<cv::Point2f> getCorners(){
        if(track_corners)
            return TheMarker;
        else
        {
            return {center};
        }
    }

    Marker& getMarker(){return TheMarker;}
    float getTrustVal() {return trust;}

    ~TrackerImpl();


private:
    void prepare(const Marker& marker, const cv::Mat& imgPyr);
    cv::Mat divDFTs( const cv::Mat &src1, const cv::Mat &src2 ) const;
    void preProcess( cv::Mat &window, cv::Mat &hw) const;
    double correlate( const cv::Mat &image_sub, cv::Point &delta_xy, const cv::Mat& Hm ) const;
    cv::Mat randWarp( const cv::Mat& a ) const;
    bool optimalScale(const cv::Mat& image, cv::Point2f& roiCenter);
    void buildCornersPyramid(const cv::Mat& image);
    void cornerUpsample(std::vector<cv::Point2f> &corners);
    Marker TheMarker;
    uint64_t TheMarkerCode;

    cv::Rect2d boundingBox;
    cv::Mat roi;            //Region of interest
    cv::Point2f center;     //Center of the marker
    cv::Mat hanWin;
    cv::Mat G;              //goal
    cv::Mat H, A, B,C;      //state

    cv::Size patchSize;
    cv::Size roiSize;
    int imgPyrIdx=0;
    bool isInit = false;
    uint32_t nbits;  //Dictionary bits
    float trust = 1; //1 is perfecf match, 0 totally different (max distance)

    double pyrfactor = 1.2;         // Pyramid scale
    int markerWarpSize;        // Desired marker size
    const double eps=0.00001;       // for normalization
    const double rate=0.2;          // learning rate
    double psrThreshold;  // no detection, if PSR is smaller than this
    const double lambda1=1e-4;      // lamda mosse
    const double lambda2=20;        // lamda contex aware
    bool contextAware=true;
    int padding = 2;
    int multiScaleLevels = 2;       // Should be even (2 mean one level up and one down)

    bool track_corners = true;
    cv::Size cornerPatchSize;

    cv::Mat cornerhanWin, Gc;
    std::vector<cv::Mat> Ac, Bc, Hc;    //state for corners
    std::vector<cv::Mat> cornersPyrs[4];    //Region corners
    double cornersPyrFactor = 4;
};
#endif // TrackerImpl_H
