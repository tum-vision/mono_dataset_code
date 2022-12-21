/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

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
********************************/
#include <iostream>
#include <fstream>
#include <sstream>
#ifdef __APPLE__
#include <GLUT/glut.h>
#elif _MSC_VER
//http://social.msdn.microsoft.com/Forums/eu/vcgeneral/thread/7d6e6fa5-afc2-4370-9a1f-991a76ccb5b7
#include <windows.h>
#include <GL/gl.h>
#include <GL/glut.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#endif

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "aruco.h"
#include "markermap.h"

void  __glGetModelViewMatrix(double modelview_matrix[16],const cv::Mat &Rvec,const cv::Mat &Tvec) throw(cv::Exception) ;
using namespace cv;
using namespace aruco;

 bool The3DInfoAvailable=false;
float TheMarkerSize=-1;
MarkerDetector MDetector;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
MarkerDetector TheMarkerDetector;
MarkerMapPoseTracker MMPoseTracker;
MarkerMap TheMMConfig;
Mat TheInputImage,TheUndInputImage,TheResizedImage,TheMask;
CameraParameters TheCameraParams;
Size TheGlWindowSize;
bool TheCaptureFlag=true;
void vDrawScene();
void vIdle();
void vResize( GLsizei iWidth, GLsizei iHeight );
void vMouse(int b,int s,int x,int y);



/************************************
 *
 *
 *
 *
 ************************************/

int main(int argc,char **argv)
{
    try
    {
        if (argc!=5) {
            cerr<<"Invalid number of arguments"<<endl;
            cerr<<"Usage: (in.avi|live) markermap_config.yml  intrinsics.yml   size "<<endl;
            cerr<<"WARNING: this test creates a synthetic mask consisting of a single rectangle. "<<endl;
            cerr<<"WARNING: The only purpose is to show how to create an AR application with mask in OpenGL "<<endl;
            return false;
        }
        TheCameraParams.readFromXMLFile(argv[3]);
        //read board configuration
        TheMMConfig.readFromFile(argv[2]);
        TheMarkerSize=atof(argv[4]);
        if (TheMMConfig.isExpressedInPixels())
            TheMMConfig=TheMMConfig.convertToMeters(TheMarkerSize);

        MMPoseTracker.setParams(TheCameraParams,TheMMConfig);
        //Open video input source
        if (string(argv[1])=="live")  //read from camera
            TheVideoCapturer.open(0);
        else TheVideoCapturer.open(argv[1]);

        if (!TheVideoCapturer.isOpened()) throw std::runtime_error("could not open video");

        //read first image
        TheVideoCapturer>>TheInputImage;
        //read camera paramters if passed
        TheCameraParams.readFromXMLFile(argv[3]);
        TheCameraParams.resize( TheInputImage.size());

        TheMarkerDetector.setThresholdParams(25,7);

        glutInit(&argc, argv);
        glutInitWindowPosition( 0, 0);
        glutInitWindowSize(TheInputImage.size().width,TheInputImage.size().height);
        glutInitDisplayMode( GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE );
        glutCreateWindow( "ArUco" );
        glutDisplayFunc( vDrawScene );
        glutIdleFunc( vIdle );
        glutReshapeFunc( vResize );
        glutMouseFunc(vMouse);
        glClearColor( 0.0, 0.0, 0.0, 1.0 );
        glClearDepth( 1.0 );

        // these two are necesary for the mask effect
        glEnable( GL_ALPHA_TEST );
        glAlphaFunc( GL_GREATER, 0.5 );

        TheGlWindowSize=TheInputImage.size();
        vResize(TheGlWindowSize.width,TheGlWindowSize.height);
        glutMainLoop();

    } catch (std::exception &ex)

    {
        cout<<"Exception :"<<ex.what()<<endl;
    }

}



/************************************
 *
 *
 *
 *
 ************************************/

cv::Mat createSyntheticMask(const cv::Mat &img) {
  // just create a mask consisting of a rectangle in the middle of the image
  // very simple but enough to show how to employ mask in opengl
  cv::Mat mask(img.size(), CV_8UC1, cv::Scalar::all(255)); // 255 means it is hidden
  cv::rectangle(mask,cv::Rect(img.cols/4, img.rows/4, img.cols/2, img.rows/2), cv::Scalar(0), CV_FILLED); //create visible (0) rectangle
  return mask;
}




/************************************
 *
 *
 *
 *
 ************************************/

cv::Mat createMultiChannelMask(const cv::Mat &img, const cv::Mat &mask)
{
  cv::Mat out(img.size(), CV_8UC4, cv::Scalar::all(0));
  for(int i=0; i<img.total(); i++) {
    for(int j=0; j<3; j++) out.ptr<cv::Vec4b>()[i][j] = img.ptr<cv::Vec3b>()[i][j];
    if(mask.size()==img.size()) out.ptr<cv::Vec4b>()[i][3] = mask.ptr<unsigned char>()[i];
  }
  return out;
}








/************************************
 *
 *
 *
 *
 ************************************/

void vMouse(int b,int s,int x,int y)
{
    if (b==GLUT_LEFT_BUTTON && s==GLUT_DOWN) {
        TheCaptureFlag=!TheCaptureFlag;
    }

}

/************************************
 *
 *
 *
 *
 ************************************/
void axis(float size)
{
    glColor3f (1,0,0 );
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f); // origin of the line
    glVertex3f(size,0.0f, 0.0f); // ending point of the line
    glEnd( );

    glColor3f ( 0,1,0 );
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f); // origin of the line
    glVertex3f( 0.0f,size, 0.0f); // ending point of the line
    glEnd( );


    glColor3f (0,0,1 );
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f); // origin of the line
    glVertex3f(0.0f, 0.0f, size); // ending point of the line
    glEnd( );


}
/************************************
 *
 *
 *
 *
 ************************************/
void vDrawScene()
{
    if (TheResizedImage.rows==0) //prevent from going on until the image is initialized
        return;
    ///clear
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    ///draw image in the buffer
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, TheGlWindowSize.width, 0, TheGlWindowSize.height, -1.0, 1.0);
    glViewport(0, 0, TheGlWindowSize.width , TheGlWindowSize.height);
    glDisable(GL_TEXTURE_2D);
    glPixelZoom( 1, -1);
    glRasterPos3f( 0, TheGlWindowSize.height  - 0.5, -1.0 );
    glDrawPixels ( TheGlWindowSize.width , TheGlWindowSize.height , GL_RGB , GL_UNSIGNED_BYTE , TheResizedImage.ptr(0) );
    ///Set the appropriate projection matrix so that rendering is done in a enrvironment
    //like the real camera (without distorsion)
    glMatrixMode(GL_PROJECTION);
    double proj_matrix[16];
    TheCameraParams.glGetProjectionMatrix(TheInputImage.size(),TheGlWindowSize,proj_matrix,0.05,10);
    glLoadIdentity();
    glLoadMatrixd(proj_matrix);
    glLineWidth(2);
    //now, for each marker,
    double modelview_matrix[16];

//         for (unsigned int m=0;m<TheMarkers.size();m++)
//         {
//             TheMarkers[m].glGetModelViewMatrix(modelview_matrix);
//             glMatrixMode(GL_MODELVIEW);
//             glLoadIdentity();
//             glLoadMatrixd(modelview_matrix);
//     // 		axis(TheMarkerSize);
//             glColor3f(1,0.4,0.4);
//             glTranslatef(0, TheMarkerSize/2,0);
//             glPushMatrix();
//             glutWireCube( TheMarkerSize );
//
//             glPopMatrix();
//         }
    //If the board is detected with enough probability
    if (!MMPoseTracker.getRTMatrix().empty()) {
        __glGetModelViewMatrix(modelview_matrix,MMPoseTracker.getRvec(),MMPoseTracker.getTvec());
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glLoadMatrixd(modelview_matrix);
        glColor3f(0,1,0);
        axis(TheMarkerSize);
        cerr<<TheMarkerSize<<endl;cin.ignore();
//        if(TheMarkerDetector.isYPerpendicular()) glTranslatef(0,TheMarkerSize/2,0);
//        else
            glTranslatef(0,0,TheMarkerSize/2);
        glPushMatrix();
        //         glutWireCube( TheMarkerSize );
        glutWireTeapot(2*TheMarkerSize);
        glPopMatrix();
    }

    // After drawing everything, now draw mask
    cv::Mat multiChannelMask = createMultiChannelMask(TheResizedImage, TheMask);
    // now redraw to create the mask effect, and thats all
    glDrawPixels ( TheGlWindowSize.width , TheGlWindowSize.height , GL_RGBA , GL_UNSIGNED_BYTE , multiChannelMask.ptr(0) );

    glutSwapBuffers();

}


/************************************
 *
 *
 *
 *
 ************************************/
void vIdle()
{
    if (TheCaptureFlag) {
        //capture image
        TheVideoCapturer.grab();
        TheVideoCapturer.retrieve( TheInputImage);
        TheUndInputImage.create(TheInputImage.size(),CV_8UC3);
        //by deafult, opencv works in BGR, so we must convert to RGB because OpenGL in windows preffer
        cv::cvtColor(TheInputImage,TheInputImage,CV_BGR2RGB);
        //remove distorion in image
        cv::undistort(TheInputImage,TheUndInputImage, TheCameraParams.CameraMatrix,TheCameraParams.Distorsion);
        //detect markers
        TheMarkers=MDetector.detect(TheUndInputImage);
        MMPoseTracker.estimatePose(TheMarkers);
        //resize the image to the size of the GL window
        cv::resize(TheUndInputImage,TheResizedImage,TheGlWindowSize);
        // create mask. It is a syntetic mask consisting of a simple rectangle, just to show a example of opengl with mask
         TheMask = createSyntheticMask(TheResizedImage); // lets create with the same size of the resized image, i.e. the size of the opengl window
    }
    glutPostRedisplay();
}


/************************************
 *
 *
 *
 *
 ************************************/
void vResize( GLsizei iWidth, GLsizei iHeight )
{
    TheGlWindowSize=Size(iWidth,iHeight);
    //not all sizes are allowed. OpenCv images have padding at the end of each line in these that are not aligned to 4 bytes
    if (iWidth*3%4!=0) {
        iWidth+=iWidth*3%4;//resize to avoid padding
        vResize(iWidth,TheGlWindowSize.height);
    }
    else {
        //resize the image to the size of the GL window
        if (TheUndInputImage.rows!=0)
            cv::resize(TheUndInputImage,TheResizedImage,TheGlWindowSize);
    }
}


void  __glGetModelViewMatrix(double modelview_matrix[16],const cv::Mat &Rvec,const cv::Mat &Tvec) throw(cv::Exception) {
     assert(Tvec.type()==CV_32F);
    // check if paremeters are valid
     Mat Rot(3, 3, CV_32FC1), Jacob;
    Rodrigues(Rvec, Rot, Jacob);
 
     double para[3][4];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            para[i][j] = Rot.at< float >(i, j);
    // now, add the translation
    para[0][3] = Tvec.ptr< float >(0)[0];
    para[1][3] = Tvec.ptr< float >(0)[1];
    para[2][3] = Tvec.ptr< float >(0)[2];
    double scale = 1;

    modelview_matrix[0 + 0 * 4] = para[0][0];
    // R1C2
    modelview_matrix[0 + 1 * 4] = para[0][1];
    modelview_matrix[0 + 2 * 4] = para[0][2];
    modelview_matrix[0 + 3 * 4] = para[0][3];
    // R2
    modelview_matrix[1 + 0 * 4] = para[1][0];
    modelview_matrix[1 + 1 * 4] = para[1][1];
    modelview_matrix[1 + 2 * 4] = para[1][2];
    modelview_matrix[1 + 3 * 4] = para[1][3];
    // R3
    modelview_matrix[2 + 0 * 4] = -para[2][0];
    modelview_matrix[2 + 1 * 4] = -para[2][1];
    modelview_matrix[2 + 2 * 4] = -para[2][2];
    modelview_matrix[2 + 3 * 4] = -para[2][3];
    modelview_matrix[3 + 0 * 4] = 0.0;
    modelview_matrix[3 + 1 * 4] = 0.0;
    modelview_matrix[3 + 2 * 4] = 0.0;
    modelview_matrix[3 + 3 * 4] = 1.0;
    if (scale != 0.0) {
        modelview_matrix[12] *= scale;
        modelview_matrix[13] *= scale;
        modelview_matrix[14] *= scale;
    }
}
