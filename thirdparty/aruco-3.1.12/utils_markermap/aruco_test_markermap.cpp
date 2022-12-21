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

#include "aruco.h"
#include "timers.h"
#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <string>
#include <stdexcept>
#include <Eigen/Geometry>
#include "sglviewer.h"
using namespace cv;
using namespace aruco;
using namespace std;
string TheMarkerMapConfigFile;
bool The3DInfoAvailable = false;
float TheMarkerSize = -1;
VideoCapture TheVideoCapturer;
Mat TheInputImage, TheInputImageCopy;
CameraParameters TheCameraParameters;
MarkerMap TheMarkerMapConfig;
MarkerDetector TheMarkerDetector;
MarkerMapPoseTracker TheMSPoseTracker;
 int waitTime = 0;
std::map<int, cv::Mat> frame_pose_map;  // set of poses and the frames they were detected
sgl_OpenCV_Viewer Viewer;
class CmdLineParser
{
    int argc;char** argv;public:
    CmdLineParser(int _argc, char** _argv): argc(_argc), argv(_argv){}
    //is the param?
    bool operator[](string param)
    {int idx = -1;   for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param) idx = i;return (idx != -1); }
    //return the value of a param using a default value if it is not present
    string operator()(string param, string defvalue = "-1"){int idx = -1;for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param) idx = i;if (idx == -1) return defvalue;else return (argv[idx + 1]);}
};
void savePCDFile(string fpath, const aruco::MarkerMap& ms,
                 const std::map<int, cv::Mat> frame_pose_map);
void savePosesToFile(string filename, const std::map<int, cv::Mat>& fmp);



/************************************
 *
 *
 *
 *
 ************************************/
int main(int argc, char** argv)
{
    try
    {
        CmdLineParser cml(argc, argv);
        if (argc < 4 || cml["-h"])
        {
            cerr << "Invalid number of arguments" << endl;
            cerr << "Usage: (in.avi|live[:camera_index(e.g 0 or 1)])) marksetconfig.yml camera_intrinsics.yml [optional_arguments]  "
                    "\n\t[-s marker_size] \n\t[-pcd out_pcd_file_with_camera_poses] \n\t[-poses out_file_with_poses] "
                    "\n\t[-mti value: minimum value in range (0,1) for the size of the detected markers. If 0, ] "
                    "\n\t[-config arucoConfig.yml: Loads the detector configuration from file ] "
                 << endl;
            return false;
        }
        TheMarkerMapConfig.readFromFile(argv[2]);

        TheMarkerMapConfigFile = argv[2];
        TheMarkerSize = stof(cml("-s", "1"));
        // read from camera or from  file
        string TheInputVideo=string(argv[1]);
         if ( TheInputVideo.find( "live")!=std::string::npos)
        {
            int vIdx = 0;
            // check if the :idx is here
            char cad[100];
            if (TheInputVideo.find(":") != string::npos)
            {
                std::replace(TheInputVideo.begin(), TheInputVideo.end(), ':', ' ');
                sscanf(TheInputVideo.c_str(), "%s %d", cad, &vIdx);
            }
            cout << "Opening camera index " << vIdx << endl;
            TheVideoCapturer.open(vIdx);
            waitTime = 10;
        }
        else
            TheVideoCapturer.open(argv[1]);        // check video is open
        if (!TheVideoCapturer.isOpened())
            throw std::runtime_error("Could not open video");

        // read first image to get the dimensions
        TheVideoCapturer >> TheInputImage;

        // read camera parameters if passed
        TheCameraParameters.readFromXMLFile(argv[3]);
        TheCameraParameters.resize(TheInputImage.size());
        // prepare the detector

        TheMarkerDetector.setDictionary( TheMarkerMapConfig.getDictionary());

        if (cml["-config"])
            TheMarkerDetector.loadParamsFromFile(cml("-config"));
         // prepare the pose tracker if possible
        // if the camera parameers are avaiable, and the markerset can be expressed in meters, then go

        if (TheMarkerMapConfig.isExpressedInPixels() && TheMarkerSize > 0)
            TheMarkerMapConfig = TheMarkerMapConfig.convertToMeters(TheMarkerSize);

        cout << "TheCameraParameters.isValid()=" << TheCameraParameters.isValid() << " "<< TheMarkerMapConfig.isExpressedInMeters() << endl;

        if (TheCameraParameters.isValid() && TheMarkerMapConfig.isExpressedInMeters()){
            TheMSPoseTracker.setParams(TheCameraParameters, TheMarkerMapConfig);
            TheMarkerSize=cv::norm(TheMarkerMapConfig[0][0]- TheMarkerMapConfig[0][1]);
        }

        // Create gui

        Viewer.setParams(1.5,1280,960,"map_viewer",TheMarkerSize);
        char key = 0;
        int index = 0;
        // capture until press ESC or until the end of the video
        cout << "Press 's' to start/stop video" << endl;
        Timer avrgTimer;
        do
        {
            TheVideoCapturer.retrieve(TheInputImage);
            TheInputImage.copyTo(TheInputImageCopy);
            index++;  // number of images captured
            if (index>1) avrgTimer.start();//do not consider first frame that requires initialization
            // Detection of the markers
            vector<aruco::Marker> detected_markers = TheMarkerDetector.detect(TheInputImage);
            // estimate 3d camera pose if possible
            if (TheMSPoseTracker.isValid())
                if (TheMSPoseTracker.estimatePose(detected_markers)) {
                    frame_pose_map.insert(make_pair(index, TheMSPoseTracker.getRTMatrix()));
                    cout<<TheMSPoseTracker.getRvec()<<" "<< TheMSPoseTracker.getTvec()<<endl;
                }
            if (index>1) avrgTimer.end();
            cout<<"Average computing time "<<avrgTimer.getAverage()<<" ms"<<endl;
            // print the markers detected that belongs to the markerset
            for (auto idx : TheMarkerMapConfig.getIndices(detected_markers))
                detected_markers[idx].draw(TheInputImageCopy, Scalar(0, 0, 255), 1);

            // show  informatino
            key =     Viewer.show(TheMarkerMapConfig,TheMSPoseTracker.getRTMatrix(),TheInputImageCopy,waitTime);
            if (key=='s') waitTime=waitTime?0:10;

        } while (key != 27 && TheVideoCapturer.grab());

        // save a beatiful pcd file (pcl library) showing the results (you can use pcl_viewer to see it)
        if (cml["-pcd"])
            savePCDFile(cml("-pcd"), TheMarkerMapConfig, frame_pose_map);

        // save the poses to a file in tum rgbd data format
        if (cml["-poses"])
            savePosesToFile(cml("-poses"), frame_pose_map);
    }
    catch (std::exception& ex)

    {
        cout << "Exception :" << ex.what() << endl;
    }
}



void  getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,double &qx,double &qy,double &qz,double &qw,double &tx,double &ty,double &tz){
    //get the 3d part of matrix and get quaternion
    assert(M_in.total()==16);
    cv::Mat M;
    M_in.convertTo(M,CV_64F);
    cv::Mat r33=cv::Mat ( M,cv::Rect ( 0,0,3,3 ) );
    //use now eigen
    Eigen::Matrix3f e_r33;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            e_r33(i,j)=M.at<double>(i,j);

    //now, move to a angle axis
    Eigen::Quaternionf q(e_r33);
    qx=q.x();
    qy=q.y();
    qz=q.z();
    qw=q.w();


    tx=M.at<double>(0,3);
    ty=M.at<double>(1,3);
    tz=M.at<double>(2,3);
}

void savePosesToFile(string filename, const std::map<int, cv::Mat>& fmp)
{
    std::ofstream file(filename);
    double qx, qy, qz, qw, tx, ty, tz;
    for (auto frame : fmp)
    {
        if (!frame.second.empty())
        {
            cv::Mat minv=frame.second.inv();
            getQuaternionAndTranslationfromMatrix44(minv, qx, qy, qz, qw, tx, ty, tz);
            file << frame.first << " " << tx << " " << ty << " " << tz << " " << qx << " " << qy << " " << qz << " "
                 << qw << endl;
        }
    }
}
