#include "fractallabelers/fractalmarkerset.h"
#include "dictionary.h"
#include <cstdio>
#include <opencv2/highgui/highgui.hpp>
#include <string>

using namespace std;

class CmdLineParser
{
    int argc;char** argv;public:
    CmdLineParser(int _argc, char** _argv): argc(_argc), argv(_argv){}
    bool operator[](string param)
    {int idx = -1;   for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param) idx = i;return (idx != -1); }
    string operator()(string param, string defvalue = "-1"){int idx = -1;for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param) idx = i;if (idx == -1) return defvalue;else return (argv[idx + 1]);}
};

std::vector<std::pair<int,int>> getRegionsConfig(string configuration){
    if (configuration.empty())return {};
    for(auto &c:configuration) if (c==',') c=' ';
    stringstream sstr(configuration);
    string markerConfig;
    std::vector<std::pair<int,int>> n_k;
    while(!sstr.eof()){
        if (sstr>>markerConfig)
        {
            int nVal, kVal;
            if (sscanf(markerConfig.c_str(), "%d:%d", &nVal, &kVal) != 2)
            {
                cerr << "Incorrect N:K specification" << endl;
                return {};
            }

            if(nVal <= kVal)
            {
                cerr << "Incorrect N:K specification. N should be > than K" << endl;
                return {};
            }

            n_k.push_back(make_pair(nVal,kVal));
        }
    }

    return n_k;
}

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    try
    {
        CmdLineParser cml(argc, argv);
        if (argc < 3 || cml["-h"])
        {
            cerr << "Usage: fractal_config.yml "
                    "n(f1):k(f1),n(f2):k(f2),...,n(fm),k(fm) "
                    "[-s bitSize (For the last level, in px. Default: -1, normalized marker)>]" << endl;

            cerr << endl;
            return -1;
        }

        //n(f1):k(f1),n(f2):k(f2),...,n(fm),k(fm)
        //Example fractal marker with 3 levels -> 10:8,14:10,6:0
        std::vector<std::pair<int,int>> regionsConfig;
        regionsConfig = getRegionsConfig(argv[2]);
        if(regionsConfig.size()<1) return -1;

        //bixSize (last level)
        int bitSize = stoi(cml("-s", "-1"));

        //Create configuration
        aruco::FractalMarkerSet fractalmarkerset;
        fractalmarkerset.create(regionsConfig, bitSize);

        //Save configuration file
        cv::FileStorage fs(argv[1], cv::FileStorage::WRITE);
        fractalmarkerset.saveToFile(fs);
    }
    catch (std::exception& ex)
    {
        cout << ex.what() << endl;
    }
}
