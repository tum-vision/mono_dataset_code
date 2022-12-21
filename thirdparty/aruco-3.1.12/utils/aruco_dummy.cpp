#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "markerdetector.h"
#include "dictionary_based.h"
using namespace  std;

std::vector<cv::Point2f>  getM0InnerCorners(vector<cv::Point2f> marker0Corners)
{
    auto homotransform=[](cv::Point2f p,cv::Mat H){
      double *h=H.ptr<double>();
      cv::Point3f res;
      res.x=p.x*h[0]+p.y*h[1]+h[2];
      res.y=p.x*h[3]+p.y*h[4]+h[5];
      res.z=p.x*h[6]+p.y*h[7]+h[8];
      return cv::Point2f(res.x/res.z,res.y/res.z);
    };
    uchar m0[64]= { 0, 0,0,0,0,0,0, 0,
                  0, 1,1,0,1,0,0, 0,
                  0, 1,0,1,0,1,1, 0,
                  0, 0,1,1,0,0,0, 0,
                  0, 1,1,1,0,1,0, 0,
                  0, 0,0,0,0,1,0, 0,
                  0, 0,1,1,1,0,1, 0,
                  0, 0,0,0,0,0,0, 0
           };
    cv::Mat mat(8,8,CV_8UC1,m0);
    int nBitsSquared = int(sqrt(mat.total()));
    float bitSize =  1. / (nBitsSquared);

    //Get inner corners
    std::vector<cv::Point2f> innerCorners;
    for(int y=0; y< mat.rows-1; y++)
    {
        for(int x=0; x< mat.cols-1; x++)
        {

            if(     ((mat.at<uchar>(y, x) == mat.at<uchar>(y+1, x+1)) &&
                     (mat.at<uchar>(y, x) != mat.at<uchar>(y, x+1) ||
                     mat.at<uchar>(y, x) != mat.at<uchar>(y+1, x)))

                    ||

                    ((mat.at<uchar>(y, x+1) == mat.at<uchar>(y+1, x)) &&
                    (mat.at<uchar>(y, x+1) != mat.at<uchar>(y, x) ||
                     mat.at<uchar>(y, x+1) != mat.at<uchar>(y+1, x+1)))
                )
                innerCorners.push_back(cv::Point2f(1+x-nBitsSquared/2.f, -(1+y-nBitsSquared/2.f)) * bitSize);
        }
    }

    std::vector<cv::Point2f> borderCorners;


    borderCorners.emplace_back( -0.5,0.5);
    borderCorners.emplace_back( 0.5,0.5);
    borderCorners.emplace_back( 0.5,-0.5);
    borderCorners.emplace_back( -0.5,-0.5);


    cv::Mat H=findHomography(borderCorners,marker0Corners);
    //now compute the homography for the inner coners
     for(auto &c:innerCorners)
        c=homotransform(c,H);

    return innerCorners;
}

int main(int argc,char **argv){
    try {
        if(argc==1)throw std::runtime_error("Usage: image");
        aruco::MarkerDetector Detector;
        auto img=cv::imread(argv[1]);
        cv::Mat grey;
        cv::cvtColor(img,grey,cv::COLOR_BGRA2GRAY);

        if(img.empty())throw  std::runtime_error ("Bad image");
        auto markers=Detector.detect(img);
        cv::Point2f pOff (5,5);

        aruco::DictionaryBased *db=dynamic_cast<aruco::DictionaryBased*>( Detector.getMarkerLabeler().get());
        if(db!=nullptr){
            auto mc=db->getDictionaries()[0].getMapCode();
            cout<<mc[0]<<endl;
        }

        for(auto m:markers){
            if(m.id==0){
                auto corners=getM0InnerCorners( m );
                cv::cornerSubPix(grey,corners,cv::Size(5,5) ,cv::Size(-1, -1),cv::TermCriteria( cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 12, 0.005));
                cout<<corners.size()<<endl;
                for(auto c:corners)
                    cv::rectangle(img,c-pOff,c+pOff,cv::Scalar(0,0,255) ,-1);
                for(auto c:m)
                    cv::rectangle(img,c-pOff,c+pOff,cv::Scalar(255,255,0) ,-1);

             }
        }

        cv::Mat res;
        cv::resize(img,res,cv::Size(800,600));
        cv::imshow("image",res);cv::waitKey(0);


    } catch (std::exception &ex) {
        cerr<<ex.what()<<endl;
    }
}

