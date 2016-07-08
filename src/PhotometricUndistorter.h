#include <opencv2/core/core.hpp>
#include "Eigen/Core"




class PhotometricUndistorter
{
public:
	PhotometricUndistorter(std::string file,std::string vignetteImage,int w_, int h_);
	~PhotometricUndistorter();

	void unMapImage(unsigned char* image_in, float* image_out, int n, bool undoGamma, bool undoVignette, bool killOverexposed);
	float* getGInv()  {if(!validGamma) return 0; else return GInv;};
	float* getG() 	    {if(!validGamma) return 0; else return G;};
private:
	float G[256];
	float GInv[256];
	float* vignetteMap;
	float* vignetteMapInv;
	int w,h;
	bool validVignette;
	bool validGamma;
};
