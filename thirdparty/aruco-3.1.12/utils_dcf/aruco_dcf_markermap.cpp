#include "aruco_cvversioning.h"
#include "aruco.h"
#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <string>
#include <stdexcept>
#include <Eigen/Geometry>
#include "sglviewer.h"
#include "pcdwriter.cpp"

#include "dcf/dcfmarkermaptracker.h"

using namespace cv;
using namespace aruco;
using namespace std;

string TheMarkerMapConfigFile;
float TheMarkerSize = -1;
VideoCapture TheVideoCapturer;
Mat TheInputImage;
CameraParameters TheCameraParameters;
MarkerMap TheMarkerMapConfig;

DFCMarkerMapTracker TheTracker;

struct   TimerAvrg{std::vector<double> times;size_t curr=0,n; std::chrono::high_resolution_clock::time_point begin,end;   TimerAvrg(int _n=30){n=_n;times.reserve(n);   }inline void start(){begin= std::chrono::high_resolution_clock::now();    }inline void stop(){end= std::chrono::high_resolution_clock::now();double duration=double(std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())*1e-6;if ( times.size()<n) times.push_back(duration);else{ times[curr]=duration; curr++;if (curr>=times.size()) curr=0;}}double getAvrg(){double sum=0;for(auto t:times) sum+=t;return sum/double(times.size());}};
TimerAvrg timer;

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
            cerr << "Usage: (in.avi|live[:camera_index(e.g 0 or 1)])) markersetconfig.yml camera_intrinsics.yml [optional_arguments]  "
                    "\n\t[-s marker_size] \n\t[-pcd out_pcd_file_with_camera_poses] \n\t[-poses out_file_with_poses] "
                    "\n\t[-f arucoConfig.yml: Loads the detector configuration from file ] "
                 << endl;
            return 0;
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

        // dictionary
        TheTracker.setDictionary( TheMarkerMapConfig.getDictionary());

        // set config parameters
        if (cml["-f"])
            TheTracker.loadParamsFromFile(cml("-f"));

         // prepare the pose tracker if possible
        // if the camera parameers are avaiable, and the markerset can be expressed in meters, then go
        if (TheMarkerMapConfig.isExpressedInPixels() && TheMarkerSize > 0)
            TheMarkerMapConfig = TheMarkerMapConfig.convertToMeters(TheMarkerSize);

        if (TheCameraParameters.isValid() && TheMarkerMapConfig.isExpressedInMeters()){
            TheTracker.setParams(TheCameraParameters, TheMarkerMapConfig);
            TheMarkerSize=cv::norm(TheMarkerMapConfig[0][0]- TheMarkerMapConfig[0][1]);
        }

        // Create gui
        Viewer.setParams(1.5,1280,960,"map_viewer",TheMarkerSize);

        char key = 0;
        // capture until press ESC or until the end of the video
        //cout << "Press 's' to start/stop video" << endl;

        int frameid =std::stoi(cml("-fstart", "0"));
        int frameStop =std::stoi(cml("-fstop", "-1"));

        TheVideoCapturer.set(CV_CAP_PROP_POS_FRAMES,frameid);

        int maxNFrames=TheVideoCapturer.get(CV_CAP_PROP_FRAME_COUNT)-1;


        TheVideoCapturer.set(CV_CAP_PROP_POS_FRAMES,frameid);
        do
        {
            if (frameStop == frameid) break;

            TheVideoCapturer.retrieve(TheInputImage);

            timer.start();
                // Detection and tracking of the markers
                TheTracker.track(TheInputImage);
                bool pose = TheTracker.estimatePose();
            timer.stop();

            std::cout <<"|@ Frame:"<<frameid++<<"/"+to_string(maxNFrames)<<", fps:"<<1./timer.getAvrg() << std::endl;

            // show  informatino
            if (pose){
                frame_pose_map.insert(make_pair(frameid, TheTracker.getRTMatrix()));
                TheTracker.drawMarkers(TheInputImage);
            }
            key = Viewer.show(TheMarkerMapConfig,TheTracker.getRTMatrix(),TheInputImage,waitTime);
            if (key=='s') waitTime=waitTime?0:10;


        } while (key != 27 && TheVideoCapturer.grab());

        // save pcd file
        if (cml["-pcd"])
            savePCDFile(cml("-pcd"), TheMarkerMapConfig, frame_pose_map);

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
