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

// saves to file an image of the  indicated marker from the dictionary indicated

#include "aruco.h"
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
using namespace std;

// convinience command line parser
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

int main(int argc, char** argv)
{
    try
    {        CmdLineParser cml(argc, argv);

        if (argc < 2)
        {
            // You can also use ids 2000-2007 but it is not safe since there are a lot of false positives.
            cerr << "Usage: <makerid> outfile.(jpg|png|ppm|bmp)  [options] \n\t[-e use enclsing corners]\n\t[-bs <size>:bit size in pixels. 50 by "
                    "default ] \n\t[-d <dictionary>: ARUCO_MIP_36h12 default] \n\t[-border: adds the white border around]"
                    "\n\t [-center:  highlights the center] "
                 << endl;
            auto dict_names = aruco::Dictionary::getDicTypes();
            cerr << "\t\tDictionaries: ";
            for (auto dict : dict_names)
                cerr << dict << " ";
            cerr << endl;
            cerr << "\t Instead of these, you can directly indicate the path to a file with your own generated "
                    "dictionary"
                 << endl;

            return -1;
        }
        int pixSize = std::stoi(cml("-bs", "75"));  // pixel size each each bit
        bool enclosingCorners = cml["-e"];
        bool waterMark = true;
        // loads the desired dictionary
        aruco::Dictionary dic = aruco::Dictionary::load(cml("-d", "ARUCO_MIP_36h12"));

        cv::imwrite(argv[2], dic.getMarkerImage_id(stoi(argv[1]), pixSize, waterMark, enclosingCorners,cml["-border"],cml["-center"]));
    }
    catch (std::exception& ex)
    {
        cout << ex.what() << endl;
    }
}
