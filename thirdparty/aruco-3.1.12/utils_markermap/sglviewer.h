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



///---------------------------------------------------------------------------------------------
#ifndef _SGL_OpenCV_VIewer_H
#define _SGL_OpenCV_VIewer_H
#include "sgl.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "markermap.h"
#include <string>
#include <thread>
#include <map>
 namespace aruco{
 //Class using an opencv window to render and manipulate a sgl scene
 class sgl_OpenCV_Viewer  {
     sgl::Scene _Scene;
     std::string _wname;
     float _f;
     int _w,_h;
     cv::Mat _imshow;
     int mode=1;//mode 0:big 3d, 1 big image
     float _subw_nsize=0.3;
     bool showingHelp=false;
     cv::Mat _cameraImage;
     bool showNumbers=true;
     aruco::MarkerMap  _mmap;
     float cameraSize=0.1;
     std::map<uint32_t,std::vector<sgl::Point3> > marker_points;
     bool canLeave=true;
     cv::Mat _resizedInImage;
     cv::Mat _camPose;
     bool followCamera=false;
 public:


     sgl_OpenCV_Viewer(){

     }



    void setParams(float f,int width,int height,std::string wname,float CameraSize=0.1){
        _imshow.create(height,width,CV_8UC3);
        _f=f;
        _w=width;_h=height;
        _wname=wname;
        cv::namedWindow(_wname,cv::WINDOW_NORMAL);
        cv::resizeWindow(_wname,width,height);
        cv::setMouseCallback(_wname, &sgl_OpenCV_Viewer::mouseCallBackFunc , this);

        cameraSize=CameraSize;
        sgl::Matrix44 cam;
        cam.translate({0,4,0});
        cam.rotateX(3.1415/2.);
        _Scene.setViewMatrix(cam);

    }

    int show( aruco::MarkerMap &mmap,const cv::Mat &cameraPose,const cv::Mat &cameraImage,int waitKeyTime=0){
        _cameraImage=cameraImage;
        _mmap=mmap;

        //set the marker points
        for(auto m:mmap)
            if(marker_points.count(m.id)==0)
                marker_points[m.id]=getMarkerIdPcd(m,0.5);

        if (!cameraPose.empty()) _camPose=cameraPose.inv();
        else _camPose=cv::Mat();

        //first creation of the image

        createImage();


        int k;
        bool isUsedKey=false;
        do{
            cv::imshow(_wname,_imshow);
            k=cv::waitKey(waitKeyTime);
            isUsedKey=true;
            if (k!=255) std::cout<<k<<std::endl;
            bool update=false,create=false;
            //change mode
            if (k=='m'){
                mode=mode==0?1:0;
                create=true;
            }
            else if(k=='h'){
                showingHelp=!showingHelp;
                update=true;
            }
            else if(k=='n'){
                showNumbers=!showNumbers;
                update=true;
            }
            //  else if (k=='s') canLeave=!canLeave;
            else if (k=='c'){
                followCamera=!followCamera;
                if(!followCamera){
                    sgl::Matrix44 cam;
                    cam.translate({0,4,0});
                    cam.rotateX(3.1415/2.);
                    _Scene.setViewMatrix(cam);
                }
                else _Scene.setViewMatrix(sgl::Matrix44());
                update=true;

            }
            else isUsedKey=false;
            if( update  ) updateImage();
            if (create)createImage();
        } while( isUsedKey);
        return k;
    }

    void setSubWindowSize(float normsize=0.3){
        _subw_nsize=normsize;
    }

protected:
     void  createImage(  ) ;
     void updateImage(  );
     void drawScene();
     void printHelp();
     float rigidBodyTransformation_Horn1987(const std::vector<cv::Point3f>& POrg, const std::vector<cv::Point3f>& PDst, cv::Mat& RT_4x4);


private:

     std::vector<sgl::Point3> getMarkerIdPcd(aruco::Marker3DInfo &minfo , float perct);

    struct mouseInfo{
        sgl::Point2 pos;
         bool isTranslating=false,isZooming=false,isRotating=false;
    }mi;


    static   void mouseCallBackFunc(int event, int x, int y, int flags, void* userdata){
        sgl_OpenCV_Viewer *Sv=(sgl_OpenCV_Viewer*)userdata;
        bool redraw=false;
        if  ( event == cv::EVENT_LBUTTONDOWN ){
            Sv->mi.isRotating=Sv->mi.isTranslating=Sv->mi.isZooming=false;
            if ( flags&cv::EVENT_FLAG_CTRLKEY)
                Sv->mi.isZooming=true;
            else if ( flags&cv::EVENT_FLAG_SHIFTKEY) Sv->mi.isTranslating=true;
            else Sv->mi.isRotating=true;
        }
        else if  ( event == cv::EVENT_MBUTTONDOWN ) Sv->mi.isTranslating=true;
        else if ( event == cv::EVENT_LBUTTONUP ) {              Sv->mi.isRotating=Sv->mi.isTranslating=Sv->mi.isZooming=false;
        }
        else if ( event == cv::EVENT_MBUTTONUP ) Sv->mi.isTranslating=false;
        else if ( event == cv::EVENT_MOUSEMOVE )
        {
            sgl::Point2  dif(Sv->    mi.pos.x-x,Sv->   mi.pos.y-y);
            sgl::Matrix44 tm;//=Sv->_Scene.getTransformMatrix();

            if (Sv->mi.isRotating){
                tm.rotateX(-float(dif.y)/100);
                tm.rotateZ(-float(dif.x)/100);
            }
            else if (Sv->mi.isZooming){
                auto vp=Sv->_Scene.getViewMatrix();
                vp.translate({0,0, float(-dif.y*0.01)});
                Sv->_Scene.setViewMatrix(vp);
                redraw=true;
            }
            else if (Sv->mi.isTranslating){
                auto vp=Sv->_Scene.getViewMatrix();
                vp.translate(sgl::Point3(float(-dif.x)/100, float(-dif.y)/100,0.f));
                Sv->_Scene.setViewMatrix(vp);
                redraw=true;
            }
            if (Sv->mi.isRotating||Sv->mi.isZooming ||Sv->mi.isTranslating)  {
                sgl::Matrix44 res= tm*Sv->_Scene.getModelMatrix() ;
                Sv->_Scene.setModelMatrix(res);
                redraw=true;
            }
        }
        Sv->mi.pos=sgl::Point2(x,y);
        if (redraw)     {
            Sv->updateImage();
            cv::imshow(Sv->_wname,Sv->_imshow);
        }

    }



};
void sgl_OpenCV_Viewer::drawScene(){
    auto drawMarker=[](sgl::Scene &Scn, const aruco::Marker3DInfo &m , int width){
        const auto &points= m.points;
        Scn.drawLine((sgl::Point3*)&points[0],(sgl::Point3*)&points[1],{0,0,255},width);
        Scn.drawLine((sgl::Point3*)&points[1],(sgl::Point3*)&points[2],{0,255,0},width);
        Scn.drawLine((sgl::Point3*)&points[2],(sgl::Point3*)&points[3],{255,0,0},width);
        Scn.drawLine((sgl::Point3*)&points[3],(sgl::Point3*)&points[0],{155,0,155},width);
    };


    auto  drawPyramid=[](sgl::Scene &Scn,float w,float h,float z,const sgl::Color &color,int width){
        Scn.drawLine( {0,0,0}, {w,h,z},color,width);
        Scn.drawLine( {0,0,0}, {w,-h,z},color,width);
        Scn.drawLine( {0,0,0}, {-w,-h,z},color,width);
        Scn.drawLine( {0,0,0}, {-w,h,z},color,width);
        Scn.drawLine( {w,h,z}, {w,-h,z},color,width);
        Scn.drawLine( {-w,h,z}, {-w,-h,z},color,width);
        Scn.drawLine( {-w,h,z}, {w,h,z},color,width);
        Scn.drawLine( {-w,-h,z}, {w,-h,z},color,width);
    };

    _Scene.clear(sgl::Color(255,255,255));
    if(followCamera && !_camPose.empty()){
        cv::Mat aa=_camPose.inv();
        _Scene.setViewMatrix(sgl::Matrix44(aa.ptr<float>(0)));
        _Scene.setModelMatrix();
    }


    for(auto m:_mmap){
        drawMarker(_Scene,m,1);
        if(showNumbers)
            _Scene.drawPoints(marker_points[m.id],{125,0,255},1);
    }
    if (!followCamera){

    //draw camera if it is possible
    if (!_camPose.empty()){
        _Scene.pushModelMatrix(sgl::Matrix44(_camPose.ptr<float>(0)));
        drawPyramid(_Scene,cameraSize,cameraSize/2,cameraSize/2,{0,0,255},1);
        _Scene.popModelMatrix();
    }
}

}

void sgl_OpenCV_Viewer::printHelp(){

    //print help commands
    if(!showingHelp)
            cv::putText(_imshow, "'h' showhelp", cv::Point(30,20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(200,200,250), 1);
    else{
        cv::putText(_imshow, "'h' hide help", cv::Point(30,20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(200,200,250), 1);
        cv::putText(_imshow, "'s' start/stop video", cv::Point(30,40), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(200,200,250), 1);
        cv::putText(_imshow, "'m' change view mode", cv::Point(30,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(200,200,250), 1);
        cv::putText(_imshow, "'MOUSE[+SHIFT|CTRL]' change view", cv::Point(30,80), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(200,200,250), 1);
        cv::putText(_imshow, "'n' show/hide marker numbers", cv::Point(30,100), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(200,200,250), 1);
        cv::putText(_imshow, "'c' camera mode on/off", cv::Point(30,120), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(200,200,250), 1);

    }
}

void sgl_OpenCV_Viewer::createImage(  ) {


        cv::Size subrectsize(_w*_subw_nsize,_h*_subw_nsize);
        if (mode==0) _Scene.setCameraParams(_f,_w,_h,_imshow.ptr<uchar>(0));
        else  _Scene.setCameraParams(_f,subrectsize.width,subrectsize.height);

        drawScene();





        //copy 3d image and color image
        if (mode==0){
            if (!_cameraImage.empty()){
                auto subrect=_imshow(cv::Range(_h-subrectsize.height,_h),cv::Range(_w-subrectsize.width,_w));
                cv::resize( _cameraImage,_resizedInImage,subrectsize);
                _resizedInImage.copyTo(subrect);
            }
        }
        else{
            if (!_cameraImage.empty()){
                cv::resize( _cameraImage,_imshow, cv::Size(_w,_h));
            }
            else _imshow.setTo(cv::Scalar::all(0));
            auto subrect=_imshow(cv::Range(_h-subrectsize.height,_h),cv::Range(_w-subrectsize.width,_w));
            cv::Mat    im3d=cv::Mat(_Scene.getHeight(),_Scene.getWidth(),CV_8UC3,_Scene.getBuffer());
            im3d.copyTo(subrect);
        }

        printHelp();

}


void sgl_OpenCV_Viewer::updateImage(  ) {



    drawScene();
    cv::Size subrectsize(_w*_subw_nsize,_h*_subw_nsize);
    auto subrect=_imshow(cv::Range(_h-subrectsize.height,_h),cv::Range(_w-subrectsize.width,_w));
    //copy 3d image and color image
    if (mode==0){
        if (!_cameraImage.empty()) _resizedInImage.copyTo(subrect);
    }
    else{
        cv::Mat    im3d=cv::Mat(_Scene.getHeight(),_Scene.getWidth(),CV_8UC3,_Scene.getBuffer());
        im3d.copyTo(subrect);
    }
    printHelp();
}

std::vector<sgl::Point3> sgl_OpenCV_Viewer::getMarkerIdPcd(aruco::Marker3DInfo &minfo, float perct=1 )
{
    auto  mult=[](const cv::Mat& m, sgl::Point3 p)
    {
        assert(m.isContinuous());
        assert(m.type()==CV_32F);
        sgl::Point3 res;
        const float* ptr = m.ptr<float>(0);
        res.x = ptr[0] * p.x + ptr[1] * p.y + ptr[2] * p.z + ptr[3];
        res.y = ptr[4] * p.x + ptr[5] * p.y + ptr[6] * p.z + ptr[7];
        res.z = ptr[8] * p.x + ptr[9] * p.y + ptr[10] * p.z + ptr[11];
        return res;
    };

    int id = minfo.id;
    // marker id as a set of points
    std::string text = std::to_string(id);
    int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 2;
    int thickness = 3;
    int baseline = 0;
    float markerSize_2 = minfo.getMarkerSize() / 2.f;
    cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
    cv::Mat img(textSize + cv::Size(0, baseline / 2), CV_8UC1, cv::Scalar::all(0));
    // center the text
    // then put the text itself
    cv::putText(img, text, cv::Point(0, textSize.height + baseline / 4), fontFace, fontScale, cv::Scalar::all(255),
                thickness, 8);
    // raster 2d points as 3d points
    std::vector<sgl::Point3> points_id;
    for (int y = 0; y < img.rows; y++)
        for (int x = 0; x < img.cols; x++)
            if (img.at<uchar>(y, x) != 0)
                points_id.push_back(sgl::Point3(  (float(x) / float(img.cols)) - 0.5f, (float(img.rows - y) / float(img.rows)) - 0.5f, 0.f));

    // now,scale
    for (auto& p : points_id)
        p = p*markerSize_2;
    // finally, translate
    auto orgPoints=aruco::Marker::get3DPoints( minfo.getMarkerSize());
    cv::Mat RT44;
    rigidBodyTransformation_Horn1987(orgPoints,minfo.points,RT44);

    for (auto& p : points_id)
        p = mult(RT44, p);


    //select only a fraction of them number of them
    if(perct!=1){
        int n_used=float(points_id.size())*(perct);
        std::random_shuffle(points_id.begin(),points_id.end());
        points_id.resize( n_used);
    }

    return points_id;
}



/**********************
 *
 *
 **********************/
float sgl_OpenCV_Viewer::rigidBodyTransformation_Horn1987(const std::vector<cv::Point3f>& POrg, const std::vector<cv::Point3f>& PDst, cv::Mat& RT_4x4)
{
    struct Quaternion
    {
        Quaternion(float q0, float q1, float q2, float q3)
        {
            q[0] = q0;
            q[1] = q1;
            q[2] = q2;
            q[3] = q3;
        }
        cv::Mat getRotation() const
        {
            cv::Mat R(3, 3, CV_32F);
            R.at<float>(0, 0) = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
            R.at<float>(0, 1) = 2.f * (q[1] * q[2] - q[0] * q[3]);
            R.at<float>(0, 2) = 2.f * (q[1] * q[3] + q[0] * q[2]);

            R.at<float>(1, 0) = 2.f * (q[1] * q[2] + q[0] * q[3]);
            R.at<float>(1, 1) = q[0] * q[0] + q[2] * q[2] - q[1] * q[1] - q[3] * q[3];
            R.at<float>(1, 2) = 2.f * (q[2] * q[3] - q[0] * q[1]);

            R.at<float>(2, 0) = 2.f * (q[1] * q[3] - q[0] * q[2]);
            R.at<float>(2, 1) = 2.f * (q[2] * q[3] + q[0] * q[1]);
            R.at<float>(2, 2) = q[0] * q[0] + q[3] * q[3] - q[1] * q[1] - q[2] * q[2];
            return R;
        }
        float q[4];
    };
    assert(POrg.size()== PDst.size());

    cv::Mat _org(POrg.size(),3,CV_32F,(float*)&POrg[0]);
    cv::Mat _dst(PDst.size(),3,CV_32F,(float*)&PDst[0]);


//    _org = _org.reshape(1);
//    _dst = _dst.reshape(1);
    cv::Mat Mu_s = cv::Mat::zeros(1, 3, CV_32F);
    cv::Mat Mu_m = cv::Mat::zeros(1, 3, CV_32F);
    //         cout<<_s<<endl<<_m<<endl;
    // calculate means
    for (int i = 0; i < _org.rows; i++)
    {
        Mu_s += _org(cv::Range(i, i + 1), cv::Range(0, 3));
        Mu_m += _dst(cv::Range(i, i + 1), cv::Range(0, 3));
    }
    // now, divide
    for (int i = 0; i < 3; i++)
    {
        Mu_s.ptr<float>(0)[i] /= float(_org.rows);
        Mu_m.ptr<float>(0)[i] /= float(_dst.rows);
    }

    // cout<<"Mu_s="<<Mu_s<<endl;
    // cout<<"Mu_m="<<Mu_m<<endl;

    cv::Mat Mu_st = Mu_s.t() * Mu_m;
    // cout<<"Mu_st="<<Mu_st<<endl;
    cv::Mat Var_sm = cv::Mat::zeros(3, 3, CV_32F);
    for (int i = 0; i < _org.rows; i++)
        Var_sm += (_org(cv::Range(i, i + 1), cv::Range(0, 3)).t() * _dst(cv::Range(i, i + 1), cv::Range(0, 3))) - Mu_st;
    //   cout<<"Var_sm="<<Var_sm<<endl;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Var_sm.at<float>(i, j) /= float(_org.rows);
    //   cout<<"Var_sm="<<Var_sm<<endl;

    cv::Mat AA = Var_sm - Var_sm.t();
    //     cout<<"AA="<<AA<<endl;
    cv::Mat A(3, 1, CV_32F);
    A.at<float>(0, 0) = AA.at<float>(1, 2);
    A.at<float>(1, 0) = AA.at<float>(2, 0);
    A.at<float>(2, 0) = AA.at<float>(0, 1);
    //     cout<<"A ="<<A <<endl;
    cv::Mat Q_Var_sm(4, 4, CV_32F);
    Q_Var_sm.at<float>(0, 0) = static_cast<float>(trace(Var_sm)[0]);
    for (int i = 1; i < 4; i++)
    {
        Q_Var_sm.at<float>(0, i) = A.ptr<float>(0)[i - 1];
        Q_Var_sm.at<float>(i, 0) = A.ptr<float>(0)[i - 1];
    }
    cv::Mat q33 = Var_sm + Var_sm.t() - (trace(Var_sm)[0] * cv::Mat::eye(3, 3, CV_32F));

    cv::Mat Q33 = Q_Var_sm(cv::Range(1, 4), cv::Range(1, 4));
    q33.copyTo(Q33);
    // cout<<"Q_Var_sm"<<endl<< Q_Var_sm<<endl;
    cv::Mat eigenvalues, eigenvectors;
    eigen(Q_Var_sm, eigenvalues, eigenvectors);
    // cout<<"EEI="<<eigenvalues<<endl;
    // cout<<"V="<<(eigenvectors.type()==CV_32F)<<" "<<eigenvectors<<endl;

    Quaternion rot(eigenvectors.at<float>(0, 0), eigenvectors.at<float>(0, 1), eigenvectors.at<float>(0, 2),
                   eigenvectors.at<float>(0, 3));
    cv::Mat RR = rot.getRotation();
    //  cout<<"RESULT="<<endl<<RR<<endl;
    cv::Mat T = Mu_m.t() - RR * Mu_s.t();
    //  cout<<"T="<<T<<endl;

    RT_4x4 = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat r33 = RT_4x4(cv::Range(0, 3), cv::Range(0, 3));
    RR.copyTo(r33);
    for (int i = 0; i < 3; i++)
        RT_4x4.at<float>(i, 3) = T.ptr<float>(0)[i];
    //  cout<<"RESS="<<RT<<endl;

    // compute the average transform error

    float err = 0;
    float* matrix = RT_4x4.ptr<float>(0);
    for (size_t i = 0; i < POrg.size(); i++)
    {
        cv::Point3f org = POrg[i];
        cv::Point3f dest_est;
        dest_est.x = matrix[0] * org.x + matrix[1] * org.y + matrix[2] * org.z + matrix[3];
        dest_est.y = matrix[4] * org.x + matrix[5] * org.y + matrix[6] * org.z + matrix[7];
        dest_est.z = matrix[8] * org.x + matrix[9] * org.y + matrix[10] * org.z + matrix[11];
        cv::Point3f dest_real = PDst[i];
        err += static_cast<float>(cv::norm(dest_est - dest_real));
    }
    return err / float(POrg.size());
    ;
}

}
#endif
