// This program converts a boardconfiguration file expressed in pixel to another one expressed in meters
#include "fractallabelers/fractalmarkerset.h"
#include <iostream>

using namespace std;
using namespace aruco;
int main(int argc, char** argv)
{
    try
    {
        if (argc < 4)
        {
            cerr << "Usage: in_configuration.yml fractal_size(meters) out_configuration.yml" << endl;
            return -1;
        }

        FractalMarkerSet BInfo;
        BInfo = FractalMarkerSet::load(argv[1]);

        //Save file
        cv::FileStorage fs(argv[3], cv::FileStorage::WRITE);
        BInfo.convertToMeters(static_cast<float>(atof(argv[2]))).saveToFile(fs);
    }
    catch (std::exception& ex)
    {
        cout << ex.what() << endl;
    }
}
