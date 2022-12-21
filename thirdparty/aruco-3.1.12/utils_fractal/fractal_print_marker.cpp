#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fractallabelers/fractalmarkerset.h>

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
            cerr << "Usage: outfile.(jpg|png|ppm|bmp) [-c <configurationFile|CONF_TYPE>:FRACTAL_2L_6 default] [-bs:bitsize (smaller marker) 75 by default]"
                 << " [-noborder: removes the white border around the marker]" << endl;
            cerr << "\tConfigurations: ";
            for (auto config : aruco::FractalMarkerSet::getConfigurations())
                cerr << config << " ";
            return -1;
        }

        aruco::FractalMarkerSet fractalmarkerSet = aruco::FractalMarkerSet::load(cml("-c","FRACTAL_2L_6"));
        int pixSize = std::stoi(cml("-bs", "75"));  // pixel size each bit from smaller marker

        cv::Mat result = fractalmarkerSet.getFractalMarkerImage(pixSize,!cml["-noborder"]);
        cv::imwrite(argv[1], result);
    }
    catch (std::exception& ex)
    {
        cout << ex.what() << endl;
    }
}
