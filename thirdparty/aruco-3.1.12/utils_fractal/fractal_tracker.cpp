#include "cvdrawingutils.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>

#include "fractaldetector.h"
#include "aruco_cvversioning.h"

using namespace std;
using namespace cv;
using namespace aruco;
struct   TimerAvrg{std::vector<double> times;size_t curr=0,n; std::chrono::high_resolution_clock::time_point begin,end;   TimerAvrg(int _n=30){n=_n;times.reserve(n);   }inline void start(){begin= std::chrono::high_resolution_clock::now();    }inline void stop(){end= std::chrono::high_resolution_clock::now();double duration=double(std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())*1e-6;if ( times.size()<n) times.push_back(duration);else{ times[curr]=duration; curr++;if (curr>=times.size()) curr=0;}}double getAvrg(){double sum=0;for(auto t:times) sum+=t;return sum/double(times.size());}};
static TimerAvrg Tdetect, Tpose;

cv::Mat __resize(const cv::Mat& in, int width)
{
    if (in.size().width <= width)
        return in;
    float yf = float(width) / float(in.size().width);
    cv::Mat im2;
    cv::resize(in, im2, cv::Size(width, static_cast<int>(in.size().height * yf)));
    return im2;
}

// class for parsing command line
class CmdLineParser{int argc;char** argv;public:CmdLineParser(int _argc, char** _argv): argc(_argc), argv(_argv){}   bool operator[](string param)    {int idx = -1;  for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param)idx = i;return (idx != -1);}    string operator()(string param, string defvalue = "-1")    {int idx = -1;for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param)idx = i;if (idx == -1)return defvalue;else return (argv[idx + 1]);}};

int main(int argc, char** argv)
{
    try
    {
        CmdLineParser cml(argc, argv);
        if (argc < 2 || cml["-h"])
        {
            cerr << "Usage: (video.avi|live[:index]) [-s fractalSize] [-cam cameraParams.yml] [-c <configuration.yml|CONFIG>:FRACTAL_2L_6 default]" << endl;
            cerr << "\tConfigurations: ";
            for (auto config : aruco::FractalMarkerSet::getConfigurations())
                cerr << config << " ";
            return 0;
        }

        aruco::CameraParameters CamParam;

        cv::Mat InImage;
        // Open input and read image
        VideoCapture vreader;
        bool isVideo=false;

        if(string(argv[1]).find("live")==std::string::npos){
            vreader.open(argv[1]);
             isVideo=true;
        }
        else{
            string livestr=argv[1];
            for(auto &c:livestr)if(c==':')c=' ';
            stringstream sstr;sstr<<livestr;
            string aux;int n=0;
            sstr>>aux>>n;
            vreader.open(n);
            if ( vreader.get(CV_CAP_PROP_FRAME_COUNT)>=2) isVideo=true;
        }


        if (vreader.isOpened())
            vreader >> InImage;
        else
        {
            cerr << "Could not open input" << endl;
            return -1;
        }

        // read camera parameters if passed
        if (cml["-cam"])
            CamParam.readFromXMLFile(cml("-cam"));

        // read marker size
        float MarkerSize = std::stof(cml("-s", "-1"));

        FractalDetector FDetector;
        FDetector.setConfiguration(cml("-c","FRACTAL_2L_6"));

        if (CamParam.isValid())
        {
            CamParam.resize(InImage.size());
            FDetector.setParams(CamParam, MarkerSize);
        }

        int frameId = 0;
        char key = 0;
        int waitTime=10;
        do
        {
            std::cout << "\r\rFrameId: " << frameId++<<std::endl;
            vreader.retrieve(InImage);

            // Ok, let's detect
            Tdetect.start();
            if(FDetector.detect(InImage))
            {
                std::cout << "Time detection: " << Tdetect.getAvrg()*1000 << " milliseconds"<<std::endl;
                Tdetect.stop();
                FDetector.drawMarkers(InImage);
            }

            //Pose estimation
            Tpose.start();
            if(FDetector.poseEstimation()){
                Tpose.stop();
                std::cout << "Time pose estimation: " << Tpose.getAvrg()*1000 << " milliseconds"<<std::endl;

                //Calc distance to marker
                cv::Mat tvec = FDetector.getTvec();
                double Z = sqrt(pow(tvec.at<double>(0,0),2) + pow(tvec.at<double>(1,0), 2) +
                         pow(tvec.at<double>(2,0),2));
                std::cout << "Distance to fractal marker: " << Z << " meters. "<<  std::endl;
                FDetector.draw3d(InImage); //3d
            }
            else
                FDetector.draw2d(InImage); //Ok, show me at least the inner corners!

            imshow("in", __resize(InImage, 1800));
            key = cv::waitKey(waitTime);  // wait for key to be pressed
            if (key == 's')
                waitTime = waitTime == 0 ? 10 : 0;

        } while (key != 27 && vreader.grab());
    }
    catch (std::exception& ex)
    {
        cout << "Exception :" << ex.what() << endl;
    }
}
