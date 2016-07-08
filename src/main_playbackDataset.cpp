#include "opencv2/opencv.hpp"


#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>


#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

#include "BenchmarkDatasetReader.h"

int main( int argc, char** argv )
{
	setlocale(LC_ALL, "");

	std::string dataset = argv[1];




	printf("Playback dataset %s!\n", dataset.c_str());

	DatasetReader* reader = new DatasetReader(dataset);

	Eigen::Matrix3f K_rect = reader->getUndistorter()->getK_rect();
	Eigen::Vector2i dim_rect = reader->getUndistorter()->getOutputDims();

	printf("Rectified Images: %d x %d. K:\n",dim_rect[0], dim_rect[1]);
	std::cout << K_rect << "\n\n";

	Eigen::Matrix3f K_org= reader->getUndistorter()->getK_org();
	Eigen::Vector2i dim_org = reader->getUndistorter()->getInputDims();
	float omega = reader->getUndistorter()->getOmega();

	printf("Original Images: %d x %d. omega=%f K:\n",dim_org[0], dim_org[1], omega);
	std::cout << K_org << "\n\n";


	if(argc > 2)
	{
		printf("Saving undistorted Dataset to here!\n");
		for(int i=0;i<reader->getNumImages();i++)
		{
			ExposureImage* I = reader->getImage(i, true, false, false, false);
			char buf[1000];
			snprintf(buf, 1000, "%05d.jpg", i);
			cv::imwrite(buf, cv::Mat(I->h, I->w, CV_32F, I->image));
			delete I;
		}
		exit(0);
	}






	bool autoPlay = false;
	bool rect = false;
	bool removeGamma = false;
	bool removeVignette = false;
	bool killOverexposed = false;


	for(int i=0;i<reader->getNumImages();i++)
	{
		while(true)
		{
			ExposureImage* I = reader->getImage(i, rect, removeGamma, removeVignette, killOverexposed);
			cv::imshow("Image", cv::Mat(I->h, I->w, CV_32F, I->image) * (1/255.0f));
			printf("Read image %d, time %.5f, exposure %.5fms. Rect (r): %d, remove gamma (g) %d, remove vignette (v): %d, kill overesposed (o)%d\n",
					I->id, I->timestamp, I->exposure_time,
					(int)rect, (int)removeGamma, (int)removeVignette, (int)killOverexposed);


			char k;
			if(autoPlay) k = cv::waitKey(1);
			else k = cv::waitKey(0);

			if(k=='w' || k == 'W') cv::imwrite("img.png", cv::Mat(I->h, I->w, CV_32F, I->image));

			delete I;


			if(k==' ') break;
			if(k=='s' || k == 'S') {i+=30; break;};
			if(k=='a' || k == 'A') autoPlay=!autoPlay;
			if(k=='v' || k == 'V') removeVignette=!removeVignette;
			if(k=='g' || k == 'G') removeGamma=!removeGamma;
			if(k=='o' || k == 'O') killOverexposed=!killOverexposed;
			if(k=='r' || k == 'R') rect=!rect;
			if(autoPlay) break;
		}
	}

	delete reader;
}

