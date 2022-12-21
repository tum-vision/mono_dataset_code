#include "dcfmarkermaptracker.h"
#include "aruco.h"
#include "levmarq.h"
#include "ippe.h"
#include "dictionary_based.h"
#include <opencv2/video/tracking.hpp>
#include "dcf_utils.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "aruco_cvversioning.h"
namespace aruco {

cv::Mat impl__aruco_getRTMatrix(const cv::Mat& _rvec, const cv::Mat& _tvec)
{
    assert(_rvec.type()==CV_32F && _rvec.total()==3);
    assert(_tvec.type()==CV_32F && _tvec.total()==3);

    cv::Mat Matrix(4,4,CV_32F);
    float *rt_44=Matrix.ptr<float>(0);
    //makes a fast conversion to the 4x4 array passed
         float rx=_rvec.ptr<float>(0)[0];
        float ry=_rvec.ptr<float>(0)[1];
        float rz=_rvec.ptr<float>(0)[2];
        float tx=_tvec.ptr<float>(0)[0];
        float ty=_tvec.ptr<float>(0)[1];
        float tz=_tvec.ptr<float>(0)[2];
        float nsqa=rx*rx + ry*ry + rz*rz;
        float a=std::sqrt(nsqa);
        float i_a=a?1./a:0;
        float rnx=rx*i_a;
        float rny=ry*i_a;
        float rnz=rz*i_a;
        float cos_a=cos(a);
        float sin_a=sin(a);
        float _1_cos_a=1.-cos_a;
        rt_44[0] =cos_a+rnx*rnx*_1_cos_a;
        rt_44[1]=rnx*rny*_1_cos_a- rnz*sin_a;
        rt_44[2]=rny*sin_a + rnx*rnz*_1_cos_a;
        rt_44[3]=tx;
        rt_44[4]=rnz*sin_a +rnx*rny*_1_cos_a;
        rt_44[5]=cos_a+rny*rny*_1_cos_a;
        rt_44[6]= -rnx*sin_a+ rny*rnz*_1_cos_a;
        rt_44[7]=ty;
        rt_44[8]= -rny*sin_a + rnx*rnz*_1_cos_a;
        rt_44[9]= rnx*sin_a + rny*rnz*_1_cos_a;
        rt_44[10]=cos_a+rnz*rnz*_1_cos_a;
        rt_44[11]=tz;
        rt_44[12]=rt_44[13]=rt_44[14]=0;
        rt_44[15]=1;
     return Matrix;
}

void impl__aruco_getRTfromMatrix44(const cv::Mat& M, cv::Mat& R, cv::Mat& T)
{
    assert(M.cols == M.rows && M.cols == 4);
    assert(M.type() == CV_32F || M.type() == CV_64F);
    // extract the rotation part
    cv::Mat r33 = cv::Mat(M, cv::Rect(0, 0, 3, 3));
    cv::SVD svd(r33);
    cv::Mat Rpure = svd.u * svd.vt;
    cv::Rodrigues(Rpure, R);
    T.create(1, 3, M.type());
    if (M.type() == CV_32F)
        for (int i = 0; i < 3; i++)
            T.ptr<float>(0)[i] = M.at<float>(i, 3);
    else
        for (int i = 0; i < 3; i++)
            T.ptr<double>(0)[i] = M.at<double>(i, 3);
}

    inline double hubberMono(double e){
        if (e <= 5.991) { // inlier
            return  e;
        } else  // outlier
            return  4.895303872*sqrt(e) - 5.991; // rho(e)   = 2 * delta * e^(1/2) - delta^2
    }

    inline double getHubberMonoWeight(double SqErr,double Information){
        return sqrt(hubberMono(Information * SqErr)/ SqErr);
    }

    template <typename T>
    double __aruco_solve_pnp(const std::vector<cv::Point3f>& p3d, const std::vector<cv::Point2f>& p2d, const std::vector<double>&trust_weight,
                             const cv::Mat& cam_matrix, const cv::Mat& dist, cv::Mat& r_io, cv::Mat& t_io, bool hubber=true)
    {
        assert(r_io.type() == CV_32F);
        assert(t_io.type() == CV_32F);
        assert(t_io.total() == r_io.total());
        assert(t_io.total() == 3);
        auto toSol = [](const cv::Mat& r, const cv::Mat& t) {
            typename LevMarq<T>::eVector sol(6);
            for (int i = 0; i < 3; i++)
            {
                sol(i) = r.ptr<float>(0)[i];
                sol(i + 3) = t.ptr<float>(0)[i];
            }
            return sol;
        };
        auto fromSol = [](const typename LevMarq<T>::eVector& sol, cv::Mat& r, cv::Mat& t) {
            r.create(1, 3, CV_32F);
            t.create(1, 3, CV_32F);
            for (int i = 0; i < 3; i++)
            {
                r.ptr<float>(0)[i] = sol(i);
                t.ptr<float>(0)[i] = sol(i + 3);
            }
        };

        cv::Mat Jacb;
        auto err_f = [&](const typename LevMarq<T>::eVector& sol, typename LevMarq<T>::eVector& err) {
            std::vector<cv::Point2f> p2d_rej;
            cv::Mat r, t;
            fromSol(sol, r, t);
            cv::projectPoints(p3d, r, t, cam_matrix, dist, p2d_rej, Jacb);
            err.resize(p3d.size() * 2);
            int err_idx = 0;
            for (size_t i = 0; i < p3d.size(); i++)
            {
                cv::Point2f  errP=p2d_rej[i] -p2d[i];


                if(hubber)
                {
                    double SqErr=(errP.x*errP.x+ errP.y*errP.y);

                    float robuse_weight= getHubberMonoWeight(SqErr*trust_weight[int(i/4)],1);
                    err(err_idx++) = robuse_weight* errP.x;
                    err(err_idx++) = robuse_weight* errP.y;
                }
                else
                {
                    err(err_idx++) = errP.x;
                    err(err_idx++) = errP.y;
                }
            }
        };
        auto jac_f = [&](const typename LevMarq<T>::eVector& sol, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& J) {
            (void)(sol);
            J.resize(p3d.size() * 2, 6);
            for (size_t i = 0; i < p3d.size() * 2; i++)
            {
                double* jacb = Jacb.ptr<double>(i);
                for (int j = 0; j < 6; j++)
                    J(i, j) = jacb[j];
            }
        };

        LevMarq<T> solver;
        solver.setParams(100, 0.01, 0.01);
        //  solver.verbose()=true;
        typename LevMarq<T>::eVector sol = toSol(r_io, t_io);
        auto err = solver.solve(sol, err_f, jac_f);

        fromSol(sol, r_io, t_io);
        return err;
    }


    double __hub_solve_pnp(const std::vector<cv::Point3f>& p3d, const std::vector<cv::Point2f>& p2d, const std::vector<double>&trust_weight,
                            const cv::Mat& cam_matrix, const cv::Mat& dist, cv::Mat& r_io, cv::Mat& t_io, float hubber=true)
   {
       return __aruco_solve_pnp<float>(p3d, p2d, trust_weight, cam_matrix, dist, r_io, t_io, hubber);
   }

    DFCMarkerMapTracker::DFCMarkerMapTracker()
    {

    }

    void  DFCMarkerMapTracker::loadParamsFromFile(const std::string &path)
    {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if(!fs.isOpened())throw std::runtime_error("Could not open "+path);

        if ( fs["dcf-detectRate"].type()!=cv::FileNode::NONE)
            fs["dcf-detectRate"]>>arucoDetectRate;
        if ( fs["dcf-tau_c"].type()!=cv::FileNode::NONE)
            fs["dcf-tau_c"]>>tau_c;
        if ( fs["dcf-psr"].type()!=cv::FileNode::NONE)
            fs["dcf-psr"]>>psr;
        if ( fs["dcf-tau_s"].type()!=cv::FileNode::NONE)
            fs["dcf-tau_s"]>>tau_s;

        TheDetector.loadParamsFromFile(path);
    }

    void  DFCMarkerMapTracker::setDictionary(const std::string &dict, float error_corretion_rate)
    {
        TheDetector.setDictionary(dict, error_corretion_rate);
    }

    bool DFCMarkerMapTracker::prediction(std::vector<cv::Point3f>&p3d, std::vector<cv::Point2f>&p2d)
    {
        bool updated=false;
        cv::Mat RTMatrix = impl__aruco_getRTMatrix(_rvec, _tvec);

//#define _show_prj
#ifdef _show_prj
        cv::Mat imColor;
        cv::cvtColor(grey, imColor, CV_GRAY2BGR);
#endif

        for(auto m:_map_mm)
        {
            //It is not necessary to analyze tracked markers
            if(mapMultiTracker.find(m.first) != mapMultiTracker.end())
            {
#ifdef _show_prj

                for(auto c:mapMultiTracker[m.first]->getCorners())
                    cv::circle(imColor,c,5,cv::Scalar(0,0,255),-1);
#endif
                continue;
            }

            //For a marker, number of corners that may bse visible
            int c=0;
            for(auto pt:m.second.points)
            {
                cv::Mat_<float> src(4,1,RTMatrix.type());
                src(0,0)=pt.x;src(1,0)=pt.y;src(2,0)=pt.z;src(3,0)=1;
                cv::Mat  cam_image_point = RTMatrix * src;
                cam_image_point = cam_image_point/cv::norm(cam_image_point);
                if(cam_image_point.at<float>(2,0)>0.75)
                    c++;
                else break;
            }

            if(c==4)
            {
                std::vector<cv::Point2f> prj_points;
                cv::projectPoints(m.second.points, _rvec, _tvec, _cam_params.CameraMatrix, _cam_params.Distorsion, prj_points);

                cv::Rect roi = getRectFrom4Points(prj_points, grey.size());
                if(roi.empty())
                    continue;

#ifdef _show_prj

                for(auto cc:prj_points)
                    cv::circle(imColor,cc,5,cv::Scalar(255,0,0),-1);
#endif

                //Add border
                cv::Point2f tl = cv::Point(roi.tl().x-roi.width*0.5f, roi.tl().y-roi.height*0.5f);
                if(tl.x<0) tl.x=0;
                if(tl.y<0) tl.y=0;
                cv::Point2f br = cv::Point(roi.br().x+roi.width*0.5f, roi.br().y+roi.height*0.5f);
                if(br.x>grey.size().width) br.x=grey.size().width;
                if(br.y>grey.size().height) br.y=grey.size().height;

                cv::Rect roi_border = cv::Rect(tl,br);
                cv::Mat patch = grey(roi_border);
                cv::threshold(patch, patch, 125, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(patch,contours, cv::noArray(), cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
#ifdef _show_prj
                cv::Mat patchColor;
                cv::cvtColor(patch, patchColor, CV_GRAY2BGR);
#endif
                std::vector<cv::Point2f> approxCurve;
                for(unsigned int i = 0; i < contours.size(); i++)
                {
                    cv::approxPolyDP(contours[i], approxCurve, double(contours[i].size()) * 0.05, true);
                    if (approxCurve.size() == 4 && cv::isContourConvex(approxCurve))
                    {
                        std::vector<int>corner_class;
                        cornerClassifier(patch, approxCurve, corner_class, 2);
//                        std::cout <<"corner class "<< corner_class[0]<<","<< corner_class[1]<<","<<corner_class[2]<<","<<corner_class[3] << std::endl;
//                        for(auto p:approxCurve)
//                            cv::circle(patchColor,p,2,cv::Scalar(255,0,0),-1);

                        if(corner_class[0]==0 && corner_class[1]==0 && corner_class[2]==0 && corner_class[3]==0)
                        {
//                            for(auto p:approxCurve)
//                                cv::circle(patchColor,p,2,cv::Scalar(0,0,255),-1);
                            for(auto dict:getDictionaries())
                            {
                                std::vector<uint64_t> idx4rot;
                                if(!getMarkerIds(patch, dict.nbits(), approxCurve, idx4rot))
                                    continue;
                                else
                                {
                                    uint64_t code;
                                    for(auto _code_id:dict.getMapCode())
                                        if(_code_id.second == m.first)
                                        {
                                            code =_code_id.first;
                                            break;
                                        }

                                    int hamm_dst = dict.nbits();
                                    unsigned int rotations=0;
                                    for(unsigned int r=0; r<4; r++)
                                    {
                                        int tmp = hamm_distance(idx4rot[r], code);
                                        if(tmp < hamm_dst)
                                        {
                                            hamm_dst = tmp;
                                            rotations = r;
                                        }
                                    }

                                    int _maxCorrectionAllowed = static_cast<int>( static_cast<float>(dict.tau()) * 0.6);
                                    if(hamm_dst<=_maxCorrectionAllowed)
                                    {
                                        //***
    //                                    for(int jj=0;jj<4; jj++)
    //                                        cv::putText(patchColor, std::to_string(jj),approxCurve[jj], 1,1, cv::Scalar(0,0,255),0);

    //                                    std::cout << "New tracker id "<<m.first<<std::endl;
    //                                    cv::imshow("patch",patchColor);
    //                                    cv::waitKey();
                                        //***

                                        updated = true;

                                        std::rotate(approxCurve.begin(),
                                                    approxCurve.begin() + 4 - rotations,
                                                    approxCurve.end());

                                        for(auto &ppp:approxCurve)
                                        {
                                            ppp.x +=roi_border.tl().x;
                                            ppp.y +=roi_border.tl().y;

                                            //Update 2d points vector
                                            p2d.push_back(ppp);
                                        }

                                        //Update 3d points vector
                                        for(auto pt:m.second.points)
                                            p3d.push_back(pt);

                                        mapMultiTracker[m.first] = new TrackerImpl(aruco::Marker(approxCurve, m.first), code, dict.nbits(), grey, psr, tau_s);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }


//#define _show_prj
#ifdef _show_prj
        cv::resize(imColor, imColor, cv::Size(1200,800));
        cv::imshow("IMG",imColor);
#endif
        return updated;
    }

    std::vector<Dictionary>  DFCMarkerMapTracker::getDictionaries()
    {
        return TheDetector.getMarkerLabeler().dynamicCast<DictionaryBased>()->getDictionaries();
    }

    void DFCMarkerMapTracker::track(const cv::Mat& input)
    {
        __ARUCO_ADDTIMER__;

        if (input.type() == CV_8UC3)
            cv::cvtColor(input,grey,CV_BGR2GRAY);
        else grey = input;

        __ARUCO_TIMER_EVENT__("ConvertGrey");

        std::vector<Marker> detectedMarkers;
        if(frame%arucoDetectRate == 0 || hull_factor<tau_c)
        {
            detectedMarkers=TheDetector.detect(grey);
            std::cout << "ArUco Detected: "<< detectedMarkers.size() <<" new markers"<< std::endl;
        }
        __ARUCO_TIMER_EVENT__("ArucoDetect");


        for(auto marker:detectedMarkers)
        {
            if(_map_mm.find(marker.id) != _map_mm.end())
            {
                for(auto d:getDictionaries())
                {
                    // Dictionary to which the marker belongs
                    if(d.getName() == _msconf.getDictionary())
                    {
                        //Find marker by its id
                        for(auto c_id : d.getMapCode())
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
        __ARUCO_TIMER_EVENT__("DCF_Prepare");

        //Only update no detected markers
        std::map<int, cv::Ptr<TrackerImpl>>::iterator it;
        for(it=mapMultiTracker.begin(); it!=mapMultiTracker.end(); ++it)
        {
            bool update=true;
            for(auto marker:detectedMarkers)
                if(marker.id == it->first)
                {
                    update = false;
                    break;
                }

            if(update)
                if(!it->second->updateImpl(grey))
                    mapMultiTracker.erase(it);
        }
        __ARUCO_TIMER_EVENT__("DCF_Update");


        frame ++;
    }


    bool DFCMarkerMapTracker::estimatePose()
    {
        ScopedTimerEvents Timer("aruco::DFCMarkerMapTracker::estimatePose");


        cv::Mat _prevr,_prevt;
        if (!getRvec().empty()){
            _prevr=getRvec().clone();
            _prevt=getTvec().clone();
        }

        std::vector<cv::Point2f> p2d;
        std::vector<cv::Point3f> p3d;
        std::vector<double> trust_weight;

        //Tracked markers
        for (auto tm : mapMultiTracker)
        {
            if (_map_mm.find(tm.first) != _map_mm.end())
            {
                float confidence = tm.second->getTrustVal();
                if(confidence > 0)
                {
                    std::vector<cv::Point2f> corners = tm.second->getCorners();
                    for(auto c:corners)
                        p2d.push_back(c);

                    if(corners.size() == 1)
                    {
                        cv::Point3f cent(0.f, 0.f, 0.f);
                        for (auto p : _map_mm[tm.first].points)
                        {
                            cent.x += p.x;
                            cent.y += p.y;
                            cent.z += p.z;
                        }
                        cent.x /= 4.f;
                        cent.y /= 4.f;
                        cent.z /= 4.f;

                        p3d.push_back(cent);
                    }
                    else
                    {
                        for (auto p : _map_mm[tm.first].points)
                            p3d.push_back(p);
                    }

                    trust_weight.push_back(confidence);
                }
            }
        }

        if (p2d.size() < 4)
        {
            // no points in the vector
            _rvec = cv::Mat();
            _tvec = cv::Mat();
            hull_factor=-1;

            return false;
        }
        else
        {
            if(_rvec.empty())
            {
                std::vector<Marker> vmarker;
                for (auto tm : mapMultiTracker)
                    vmarker.push_back(tm.second->getMarker());
                auto InitialPose=relocalization(vmarker);

                if(InitialPose.empty())
                {
                    hull_factor=-1;
                    return false;
                }
                impl__aruco_getRTfromMatrix44(InitialPose,_rvec,_tvec);
            }

            __hub_solve_pnp(p3d,p2d,trust_weight,_cam_params.CameraMatrix, _cam_params.Distorsion, _rvec, _tvec);

            Timer.add("PoseEstimation");
            //Update with new predicted markers!!
            if(prediction(p3d, p2d))
                __hub_solve_pnp(p3d,p2d,trust_weight,_cam_params.CameraMatrix, _cam_params.Distorsion, _rvec, _tvec);
            Timer.add("Prediction");

            std::vector<cv::Point2f> hull;
            cv::convexHull(p2d, hull);
            hull_factor=cv::contourArea(hull)/grey.size().area();

            return true;
        }
    }

    void DFCMarkerMapTracker::drawMarkers(cv::Mat& img, float alpha, float trust)
    {
        int lineSize = img.cols/700;

        for(auto m:mapMultiTracker)
        {
            //The color of the marker depends on the confidence level (threshold alpha)
            cv::Scalar color;
            if(m.second->getTrustVal() > alpha)
                color = cv::Scalar(0,0, 255);
            else
                color =cv::Scalar(255,0, 255);

            //Marker border
            m.second->getMarker().draw(img, color, lineSize, false, false);

            //Marker info (id, confidence)
            std::ostringstream txt;
            txt << m.second->getMarker().id;
            if(trust) txt <<",w:"<<m.second->getTrustVal();
            cv::putText(img,txt.str(),m.second->getCenter(),  cv::FONT_HERSHEY_SIMPLEX, 1,
                       color, lineSize);
        }
    }
}
