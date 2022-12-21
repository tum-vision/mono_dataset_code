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



//This program do batch processing on a set of images. It only prints the marker locations in 2d
#if WIN32
#include <win_dirent.h>
#else

#include <dirent.h>
#endif
#include "aruco.h"
#include "ippe.h"
#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
using namespace cv;
using namespace std;
using namespace aruco;

vector<string> readDir(string path){
    DIR *dir;
    struct dirent *ent;
    vector<string>  res;
    if ((dir = opendir (path.c_str())) != NULL) {
      /* print all the files and directories within directory */
      while ((ent = readdir (dir)) != NULL)
          res.push_back(path+string("/")+string(ent->d_name));
      closedir (dir);
    }
    return res;
}


//dist : K1,K2,K3,P1,P2,b1,b2

vector<cv::Point2f> correctDistortionFraser(const vector<cv::Point2f> &inpoints,const cv::Mat &cameraMatrix,const cv::Mat &distortionModel){
    vector<cv::Point2f> res;
    cv::Mat cm32,dm64;
    cameraMatrix.convertTo(cm32,CV_64F);
    distortionModel.convertTo(dm64,CV_64F);
    double cx=cameraMatrix.at<double>(0,2);
    double cy=cameraMatrix.at<double>(1,0);
    double f=cameraMatrix.at<double>(0,0)+cameraMatrix.at<double>(1,1);
    f*=0.5;
    double K1=distortionModel.ptr<double>(0)[0];
    double K2=distortionModel.ptr<double>(0)[1];
    double K3=distortionModel.ptr<double>(0)[2];
    double P1=distortionModel.ptr<double>(0)[3];
    double P2=distortionModel.ptr<double>(0)[4];
    double b1=distortionModel.ptr<double>(0)[5];
    double b2=distortionModel.ptr<double>(0)[6];
    for(auto p:inpoints ){
        double x,y;
        x=  (p.x-cx)/f;
        y=  (p.y-cy)/f;
        float r2=x*x + y*y;
        float r4=r2*r2;
        float r6=r4*r2;
        float ax= x *r2*K1 + x*r4*K2 +x*r6*K3 + (2*x*x+r2)*P1 + 2*P2*x*y+b1*x+b2*y;
        float ay=y*r2*K1+ y*r4*K2 +y*r6*K3+ 2*P1*x*y+ (2*y*y+r2)*P2;
        cv::Point2f pout(p);
        pout.x+=ax;
        pout.y+=ay;
         res.push_back(pout);
    }
    return res;
}

// class for parsing command line
// operator [](string cmd) return  whether cmd is present //string operator ()(string cmd) return the value as a string:
// -cmd value
class CmdLineParser
{
    int argc;
    char** argv;
public:
    CmdLineParser(int _argc, char** _argv)
          : argc(_argc)
          , argv(_argv)
    {
    }
    bool operator[](string param)
    {
        int idx = -1;
        for (int i = 0; i < argc && idx == -1; i++)
            if (string(argv[i]) == param)
                idx = i;
        return (idx != -1);
    }
    string operator()(string param, string defvalue = "-1")
    {
        int idx = -1;
        for (int i = 0; i < argc && idx == -1; i++)
            if (string(argv[i]) == param)
                idx = i;
        if (idx == -1)
            return defvalue;
        else
            return (argv[idx + 1]);
    }
};
cv::Mat __resize(const cv::Mat& in, int width)
{
    if (in.size().width <= width)
        return in;
    float yf = float(width) / float(in.size().width);
    cv::Mat im2;
    cv::resize(in, im2, cv::Size(width, static_cast<int>(in.size().height * yf)));
    return im2;
}
void parseFraserString(string str,cv::Mat &cameraMatrix,cv::Mat &DistCoeff){

for(auto &c:str)
    if (c==':') c=' ';
stringstream sstr(str);
DistCoeff.create(1,7,CV_64F);
cameraMatrix.create(3,3,CV_64F);
cameraMatrix.setTo(cv::Scalar::all(0));
sstr>>cameraMatrix.at<double>(0,2);//CX
sstr>>cameraMatrix.at<double>(1,2);//CY
sstr>>cameraMatrix.at<double>(0,0);//F
cameraMatrix.at<double>(1,1)=cameraMatrix.at<double>(0,0);
sstr>>DistCoeff.at<double>(0,0);//K1
sstr>>DistCoeff.at<double>(0,1);//K2
sstr>>DistCoeff.at<double>(0,2);//K3
sstr>>DistCoeff.at<double>(0,3);//P1
sstr>>DistCoeff.at<double>(0,4);//P2
sstr>>DistCoeff.at<double>(0,5);//b1
sstr>>DistCoeff.at<double>(0,6);//b2

}
int main(int argc, char** argv)
{
    try
    {
        CmdLineParser cml(argc, argv);
        if (argc <3 || cml["-h"])
        {
            cerr << "Usage: readDir  outFile [params]  \n"
                    "[-d <dicionary>:ALL_DICTS default]  "
                    "[-minSizeImage <val>: minimum size of a marker in the image. Range (0,1) 1 whole image] "
                    "[-Fraser CX:CY:F:K1:K2:K3:P1:P2:b1:b2] fraser calibration model"
                    "[-MSize <size>] indicates the size of the marker"
                    "[-fast] uses the fast mode"
                 << endl;
            cerr << "\tDictionaries: ";
            for (auto dict : aruco::Dictionary::getDicTypes())
                cerr << dict << " ";
            cerr << endl;
            cerr << "\t Instead of these, you can directly indicate the path to a file with your own generated "
                    "dictionary"
                 << endl;
            cout << "Example to work with apriltags dictionary : video.avi -d TAG36h11" << endl << endl;
            return 0;
        }


        // read the input image

        // Create the detector
        MarkerDetector MDetector;
        cv::Mat cameraMatrix,FraserDist;


        if (cml["-Fraser"])
              parseFraserString(cml("-Fraser"),cameraMatrix,FraserDist);
        float msize=stod(cml("-MSize","1.0"));

        aruco::DetectionMode dmode=aruco::DM_NORMAL;
        if(cml["-fast"]) dmode=aruco::DM_FAST;

         // Set the dictionary you want to work with
        // see dictionary.h for all types
        MDetector.setDictionary(cml("-d","ALL_DICTS"), 0.f);

        MDetector.setDetectionMode(dmode,msize);

        vector<string> imagesPath=readDir(argv[1]);
        for(auto ip:imagesPath){
            cv::Mat InImage=cv::imread(ip);
            if (!InImage.empty()){
                cerr<<"processing "<<ip<<endl;
                // Ok, let's detect
                vector<Marker> Markers = MDetector.detect(InImage);
                ofstream file(argv[2],ios::app);
                file<<ip<<endl;
                for(auto m:Markers) {
                    file<<m.id<< " ";
                    for(auto c:m){
                        file<<c<<" ";
                        if(!cameraMatrix.empty()){
                            for(auto &m:Markers) m=correctDistortionFraser(m,cameraMatrix,FraserDist);
                            auto solutions =  aruco::solvePnP_(msize, m,cameraMatrix,cv::Mat::zeros(1,5,CV_32F));
                            cv::Mat t=solutions[0].first.rowRange(0,3).colRange(3,4);
                            cout<<cv::norm(t)<<" ";
                        }
                    }
                    file<<endl;
                }
            }
        }
    }
    catch (std::exception& ex)

    {
        cout << "Exception :" << ex.what() << endl;
    }
}
