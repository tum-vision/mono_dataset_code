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

#include <algorithm>


#include <fstream>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include "aruco.h"
#include "aruco_calibration_grid_board_a4.h"
#include "dirreader.h"
#include <stdexcept>

using namespace std;
using namespace cv;
using namespace aruco;

Mat TheInputImage, TheInputImageCopy;
CameraParameters TheCameraParameters;
MarkerMap TheMarkerMapConfig;
MarkerDetector TheMarkerDetector;
string TheOutFile;
string TheMarkerMapConfigFile;
float TheMarkerSize = -1;
string TheOutCameraParams;
bool autoOrient=false;
// given the set of markers detected, the function determines the get the 2d-3d correspondes
void getMarker2d_3d(vector<cv::Point2f>& p2d, vector<cv::Point3f>& p3d, const vector<Marker>& markers_detected,
                    const MarkerMap& bc)
{
    p2d.clear();
    p3d.clear();
    
    // for each detected marker
    for (size_t i = 0; i < markers_detected.size(); i++)
    {
        // find it in the bc
        auto fidx = std::string::npos;
        for (size_t j = 0; j < bc.size() && fidx == std::string::npos; j++)
            if (bc[j].id == markers_detected[i].id)
                fidx = j;
        if (fidx != std::string::npos)
        {
            for (int j = 0; j < 4; j++)
            {
                p2d.push_back(markers_detected[i][j]);
                p3d.push_back(bc[fidx][j]);
            }
        }
    }
    cout << "points added" << endl;
}
vector<vector<cv::Point2f>> calib_p2d;
vector<vector<cv::Point3f>> calib_p3d;
aruco::CameraParameters camp;  // camera parameters estimated

class CmdLineParser
{  int argc;char** argv;public:
    CmdLineParser(int _argc, char** _argv): argc(_argc), argv(_argv){}
    bool operator[](string param)    {int idx = -1;   for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param) idx = i;return (idx != -1); }
    string operator()(string param, string defvalue = "-1"){int idx = -1;for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param) idx = i;if (idx == -1) return defvalue;else return (argv[idx + 1]);}
};





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
        CmdLineParser cml(argc,argv);
        if (argc < 3 || cml["-h"])
        {
            cerr << "Usage:   out_camera_calibration.yml directory_with_images [options]  " << endl;
            cerr << "options:" << endl;
            cerr << "-size maker_size : Size of the markers in meters. " << endl;
            cerr << "-m markersetconfig.yml : By default, the one in utils is assumed. Otherwise, set the file here "<<endl;
            cerr << "-auto_orient : forces width larger than height by flipping image 90degs. " << endl;
            return -1;
        }
         TheMarkerSize = stof(cml("-size","1"));
        TheMarkerMapConfigFile=cml("-m","");
        TheOutFile=argv[1];
        autoOrient=cml["-auto_orient"];

        // load board info
        if (TheMarkerMapConfigFile.empty())
        {
            stringstream sstr;
            sstr.write((char*)default_a4_board, default_a4_board_size);
            TheMarkerMapConfig.fromStream(sstr);
        }
        else
            TheMarkerMapConfig.readFromFile(TheMarkerMapConfigFile);
        // is in meters
        if (!TheMarkerMapConfig.isExpressedInMeters())
        {
            if (TheMarkerSize == -1)
            {
                cerr << "Need to specify the length of the board with -size" << endl;
                return -1;
            }
            else
                TheMarkerMapConfig = TheMarkerMapConfig.convertToMeters(TheMarkerSize);
        }

        // set specific parameters for this configuration
         TheMarkerDetector.setDictionary(TheMarkerMapConfig.getDictionary());

        vector<string> images=DirReader::read(argv[2],"",DirReader::Params(true));
        for(auto i:images)cout<<i<<endl;
        if (images.size()==0)throw std::runtime_error("Could not find a file in the speficied path");
        cv::Size imageSize(-1, -1);
        // capture until press ESC or until the end of the video
        for(size_t currImage=0;currImage<images.size();currImage++){
            cout << "reading " << images[currImage] << endl;
            if ( images[currImage].back()=='.') continue;//skip . and ..
            TheInputImage = cv::imread(images[currImage]);
            if (TheInputImage.cols<TheInputImage.rows && autoOrient){
                cv::Mat aux;
                cv::transpose(TheInputImage,aux);
                cv::flip(aux,TheInputImage,0);
            }

            if (TheInputImage.empty()) continue;//not an image
            if (imageSize != cv::Size(-1, -1) && imageSize != TheInputImage.size())
            {
                cerr << "Input image sizes must be equal" << endl;
                exit(0);
            }
            imageSize = TheInputImage.size();

            // detect and print
            vector<aruco::Marker> detected_markers = TheMarkerDetector.detect(TheInputImage);
            vector<int> markers_from_set = TheMarkerMapConfig.getIndices(detected_markers);
            TheInputImage.copyTo(TheInputImageCopy);
            for (auto idx : markers_from_set)
                detected_markers[idx].draw(TheInputImageCopy, Scalar(0, 0, 255),
                                           static_cast<int>( max(float(1.f), 1.5f * float(TheInputImageCopy.cols) / 1000.f)));

            if (TheInputImageCopy.cols > 1280)
                cv::resize(TheInputImageCopy, TheInputImage,
                           cv::Size(1280, static_cast<int>(1280.f * float(TheInputImageCopy.rows) / float(TheInputImageCopy.cols))));
            else
                TheInputImageCopy.copyTo(TheInputImage);

            vector<cv::Point2f> p2d;
            vector<cv::Point3f> p3d;

            getMarker2d_3d(p2d, p3d, detected_markers, TheMarkerMapConfig);
            if (p3d.size() > 0)
            {
                calib_p2d.push_back(p2d);
                calib_p3d.push_back(p3d);
            }

            // show input with augmented information and  the thresholded image
            cv::imshow("in", TheInputImage);
            // write to video if required
            cv::waitKey(100);  // wait for key to be pressed

        } ;

        cout << "Starting calibration" << endl;
        vector<cv::Mat> vr, vt;
        camp.CamSize = imageSize;
        cout << calib_p2d.size() << endl;
        cv::calibrateCamera(calib_p3d, calib_p2d, imageSize, camp.CameraMatrix, camp.Distorsion, vr, vt);

        //compute the average reprojection error
        std::pair<double,int> sum={0,0};
        for(size_t v=0;v<calib_p2d.size();v++){
            vector<cv::Point2f> repj;
            cv::projectPoints(calib_p3d[v],vr[v],vt[v],camp.CameraMatrix, camp.Distorsion, repj);
            for(size_t p=0;p<calib_p3d[v].size();p++){
                sum.first+=cv::norm(repj[p]-calib_p2d[v][p]);
                sum.second++;
            }
        }
        cerr << "repj error=" << sum.first/double(sum.second) << endl;



        camp.saveToFile(TheOutFile);
        
        cerr << "File saved in :" << TheOutFile << endl;
        //print locations
        cout<<"TYPE="<<(vr[0].type()==CV_64F)<<endl;
        cout<<	"std::vector<cv::Mat> vr;"<<endl;
        for(size_t i=0;i<vr.size();i++){
			double *ptr=vr[i].ptr<double>(0);
			cout<<"rv.push_back( (cv::Mat_<double>(3,1) << "<<ptr[0]<< ","<<ptr[1]<<","<<ptr[2]<<"));"<<endl;
		}
		
        cout<<	"std::vector<cv::Mat> vt;"<<endl;
        for(size_t i=0;i<vr.size();i++){
			double *ptr=vt[i].ptr<double>(0);
			cout<<"vt.push_back( (cv::Mat_<double>(3,1) << "<<ptr[0]<< ","<<ptr[1]<<","<<ptr[2]<<"));"<<endl;
		}
    }
    catch (std::exception& ex)

    {
        cout << "Exception :" << ex.what() << endl;
    }
}
