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

#ifndef _COMMON_ARUCO_
#define _COMMON_ARUCO_
#include <opencv2/core/core.hpp>
using namespace cv;
/**This function reads the matrix intrinsics and the distorsion coefficients from a file.
 * The format of the file is
 * \code
 * #  comments
 * fx fy cx cy k1 k2 p1 p2 width height 1
 * \endcode
 * @param TheIntrinsicFile path to the file with the info
 * @param TheIntriscCameraMatrix output matrix with the intrinsics
 * @param TheDistorsionCameraParams output vector with distorsion params
 * @param size of the images captured. Note that the images you are using might be different from these employed for calibration (which are in the file).
 * If so, the intrinsic must be adapted properly. That is why you must pass here the size of the images you are employing
 * @return true if params are readed properly
 */
  
bool readIntrinsicFile(string TheIntrinsicFile,Mat & TheIntriscCameraMatrix,Mat &TheDistorsionCameraParams,Size size)
{
	//open file
	ifstream InFile(TheIntrinsicFile.c_str());
	if (!InFile) return false;
	char line[1024];
	InFile.getline(line,1024);	 //skype first line that should contain only comments
	InFile.getline(line,1024);//read the line with real info

	//transfer to a proper container
	stringstream InLine;
	InLine<<line;
	//Create the matrices
	TheDistorsionCameraParams.create(4,1,CV_32FC1);
	TheIntriscCameraMatrix=Mat::eye(3,3,CV_32FC1);
	

	//read intrinsic matrix				 
	InLine>>TheIntriscCameraMatrix.at<float>(0,0);//fx								
	InLine>>TheIntriscCameraMatrix.at<float>(1,1); //fy								
	InLine>>TheIntriscCameraMatrix.at<float>(0,2); //cx								 
	InLine>>TheIntriscCameraMatrix.at<float>(1,2);//cy
	//read distorion parameters
	for(int i=0;i<4;i++) InLine>>TheDistorsionCameraParams.at<float>(i,0);
	
	//now, read the camera size
	float width,height;
	InLine>>width>>height;
	//resize the camera parameters to fit this image size
	float AxFactor= float(size.width)/ width;
	float AyFactor= float(size.height)/ height;
	TheIntriscCameraMatrix.at<float>(0,0)*=AxFactor;
	TheIntriscCameraMatrix.at<float>(0,2)*=AxFactor;
	TheIntriscCameraMatrix.at<float>(1,1)*=AyFactor;
	TheIntriscCameraMatrix.at<float>(1,2)*=AyFactor;

	//debug
	cout<<"fx="<<TheIntriscCameraMatrix.at<float>(0,0)<<endl;
	cout<<"fy="<<TheIntriscCameraMatrix.at<float>(1,1)<<endl;
	cout<<"cx="<<TheIntriscCameraMatrix.at<float>(0,2)<<endl;
	cout<<"cy="<<TheIntriscCameraMatrix.at<float>(1,2)<<endl;
	cout<<"k1="<<TheDistorsionCameraParams.at<float>(0,0)<<endl;
	cout<<"k2="<<TheDistorsionCameraParams.at<float>(1,0)<<endl;
	cout<<"p1="<<TheDistorsionCameraParams.at<float>(2,0)<<endl;
	cout<<"p2="<<TheDistorsionCameraParams.at<float>(3,0)<<endl;
	
	return true;
} 
#endif
