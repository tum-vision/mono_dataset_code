#include "dcfmarkertracker.h"
#include "aruco.h"
#include "levmarq.h" //solve pnp of opencv is not trustworthy. Create our own

#include "../ippe.h"
#include "dictionary_based.h"
#include "aruco_cvversioning.h"

namespace aruco {

    DFCMarkerTracker::DFCMarkerTracker()
    {
    }


    void DFCMarkerTracker::setParams(const aruco::CameraParameters& camPar, float mSize)
    {
        CamParam = camPar;
        MarkerSize = mSize;
    }

    std::vector<Dictionary> DFCMarkerTracker::getDictionaries()
    {
        return TheDetector.getMarkerLabeler().dynamicCast<DictionaryBased>()->getDictionaries();
    }

    void  DFCMarkerTracker::loadParamsFromFile(const std::string &path)
    {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if(!fs.isOpened())throw std::runtime_error("Could not open "+path);

        if ( fs["dcf-detectRate"].type()!=cv::FileNode::NONE)
            fs["dcf-detectRate"]>>arucoDetectRate;
        if ( fs["dcf-psr"].type()!=cv::FileNode::NONE)
            fs["dcf-psr"]>>psr;
        if ( fs["dcf-tau_s"].type()!=cv::FileNode::NONE)
            fs["dcf-tau_s"]>>tau_s;

        TheDetector.loadParamsFromFile(path);
    }

    void  DFCMarkerTracker::setDictionary(const std::string &dict, float error_corretion_rate)
    {
        TheDetector.setDictionary(dict, error_corretion_rate);
    }

    std::map<int, cv::Ptr<TrackerImpl>> DFCMarkerTracker::track(const cv::Mat& input, float thresh_confidence)
    {
        ScopedTimerEvents Timer("DFCMarkerTracker::track");

        cv::Mat grey;
        if (input.type() == CV_8UC3)
            cv::cvtColor(input,grey,CV_BGR2GRAY);
        else grey = input;

        Timer.add("ConvertGrey");

        //Aruco marker detection. Refresh list trackers.
        std::vector<Marker> detectedMarkers;
        if(frame%arucoDetectRate == 0 || mapMultiTracker.size()==0)
        {
            detectedMarkers=TheDetector.detect(grey);
            std::cout << "ArUco Detected: "<< detectedMarkers.size() <<" new markers"<< std::endl;
        }

        Timer.add("ArucoDetect");

        //Update markers
        std::map<int, cv::Ptr<TrackerImpl>>::iterator it;
        for(it=mapMultiTracker.begin(); it!=mapMultiTracker.end(); ++it)
        {
            if(!it->second->updateImpl(grey, thresh_confidence))
                mapMultiTracker.erase(it);
        }

        Timer.add("DCF_Update");

        //Initialize filters for detected markers
        std::map<int, cv::Ptr<TrackerImpl>> mapNewDetections;
        for(auto marker:detectedMarkers)
        {
            for(auto d:getDictionaries())
            {
                // Dictionary to which the marker belongs
                if(d.getName() == marker.dict_info)
                {
                    for(auto c_id : d.getMapCode())
                    {
                        //Find marker by its id
                        if(c_id.second == marker.id)
                        {
                            //New tracker!
                            mapMultiTracker[marker.id] = new TrackerImpl(marker, c_id.first, d.nbits(), grey, psr, tau_s);
                            break;
                        }
                    }
                }
            }
        }
        Timer.add("DCF_Prepare");
        frame ++;

        return mapMultiTracker;
    }

    void DFCMarkerTracker::estimatePose()
    {
        if(CamParam.isValid() && MarkerSize>0)
        {
            ScopedTimerEvents Timer("aruco::DFCMarkerTracker::estimatePose");

            for(auto m:mapMultiTracker)
                mapMultiPose[m.first].estimatePose(m.second->getMarker(), CamParam, MarkerSize);

            Timer.add("pose");
        }
    }

    void DFCMarkerTracker::draw(cv::Mat& img, cv::Scalar borderColor,cv::Scalar infoColor, bool trust)
    {
        for(auto m:mapMultiTracker)
        {
            Marker marker = m.second->getMarker();
            int lineSize = img.cols/700;

            //Marker info (id, confidence)
            std::ostringstream txt;
            txt << m.second->getMarker().id;
            if(trust) txt <<",w:"<<m.second->getTrustVal();
            cv::putText(img,txt.str(),m.second->getCenter(),  cv::FONT_HERSHEY_SIMPLEX, 2,
                       infoColor, lineSize);

            //Box and axis
            if (CamParam.isValid() && MarkerSize != -1)
            {
                if (m.second->getMarker().isPoseValid()){
                    CvDrawingUtils::draw3dCube(img, marker, CamParam, lineSize);
                    CvDrawingUtils::draw3dAxis(img, marker, CamParam, lineSize);
                }
            }
            else
            {
                //Marker border
                marker.draw(img, borderColor, lineSize, false, false);
            }
        }
    }
}
