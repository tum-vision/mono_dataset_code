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
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <stdexcept>

using namespace cv;
using namespace std;
using namespace aruco;
// class for parsing command line
// operator [](string cmd) return  whether cmd is present //string operator ()(string cmd) return the value as a string:
// -cmd value
class CmdLineParser{int argc;char** argv;public:CmdLineParser(int _argc, char** _argv): argc(_argc), argv(_argv){}   bool operator[](string param)    {int idx = -1;  for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param)idx = i;return (idx != -1);}    string operator()(string param, string defvalue = "-1")    {int idx = -1;for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param)idx = i;if (idx == -1)return defvalue;else return (argv[idx + 1]);}};

cv::Mat __resize(const cv::Mat& in, int width)
{
    if (in.size().width <= width)
        return in;
    float yf = float(width) / float(in.size().width);
    cv::Mat im2;
    cv::resize(in, im2, cv::Size(width, static_cast<int>(in.size().height * yf)));
    return im2;
}

int main(int argc, char** argv)
{
    try
    {
        CmdLineParser cml(argc, argv);
        if (argc == 1 || cml["-h"])
        {
            cerr << "Usage: (in_image|video.avi) [-c cameraParams.yml] [-s markerSize] [-d <dicionary>:ALL_DICTS default] [-f arucoConfig.yml] "   << endl;
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

        aruco::CameraParameters CamParam;
        // read the input image
        cv::Mat InImage;
        // Open input and read image
        VideoCapture vreader(argv[1]);
        if (vreader.isOpened()) vreader >> InImage;
        else throw std::runtime_error("Could not open input");

        // read camera parameters if specifed
        if (cml["-c"])
            CamParam.readFromXMLFile(cml("-c"));

        // read marker size if specified (default value -1)
        float MarkerSize = std::stof(cml("-s", "-1"));
        // Create the detector
        MarkerDetector MDetector;
         if(cml["-f"]){//uses a configuration file. YOu can create it from aruco_test application
                MDetector.loadParamsFromFile(cml("-f"));
        }
        else{

            // Set the dictionary you want to work with, if you included option -d in command line
           //By default, all valid dictionaries are examined
            if (cml["-d"])
                MDetector.setDictionary(cml("-d"), 0.f);

        }
        // Ok, let's detect
        vector<Marker> Markers = MDetector.detect(InImage, CamParam, MarkerSize);

        // for each marker, draw info and its boundaries in the image
        for (unsigned int i = 0; i < Markers.size(); i++)
        {
            cout << Markers[i] << endl;
            Markers[i].draw(InImage, Scalar(0, 0, 255), 2);
        }
        // draw a 3d cube in each marker if there is 3d info
        if (CamParam.isValid() && MarkerSize != -1)
            for (unsigned int i = 0; i < Markers.size(); i++)
            {
                if(Markers[i].id==229 || Markers[i].id==161)
                    cout<< "Camera Location= "<<Markers[i].id<<" "<<CamParam.getCameraLocation(Markers[i].Rvec,Markers[i].Tvec)<<endl;
                CvDrawingUtils::draw3dAxis(InImage, Markers[i], CamParam);
              //  CvDrawingUtils::draw3dCube(InImage, Markers[i], CamParam);
            }
        // show input with augmented information
        cv::namedWindow("in", 1);
        cv::imshow("in", __resize(InImage,1280));
        while (char(cv::waitKey(0)) != 27)
            ;  // wait for esc to be pressed

    }
    catch (std::exception& ex)

    {
        cout << "Exception :" << ex.what() << endl;
    }
}




