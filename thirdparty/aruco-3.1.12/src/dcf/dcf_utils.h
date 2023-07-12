#ifndef DFC_UTILS_H
#define DFC_UTILS_H

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <map>
#include <string>

bool getMarkerIds(const cv::Mat& im, int total_bits, std::vector<cv::Point2f>& marker_corners, std::vector<uint64_t>& ids);
float getSubpixelValue(const cv::Mat &im_grey,const cv::Point2f &p);
uint64_t touulong(const cv::Mat& code);
cv::Mat rotate(const cv::Mat& in);
int hamm_distance(uint64_t a, uint64_t b);
cv::Rect getRectFrom4Points(std::vector<cv::Point2f>points, cv::Size imSize=cv::Size(), bool imFit=false);
void cornerClassifier(const cv::Mat &im, std::vector<cv::Point2f> points,std::vector<int>& corner_class, int wsize);


struct FrameMatchLocation{
     cv::Mat first;
     cv::Mat second;
     std::string frame;
     double error;
};

cv::Mat rigidBodyTransformation_Horn1987 (const std::vector<cv::Point3d> &org, const std::vector<cv::Point3d> &dst,bool mbFixScale);
cv::Mat  alignAndScaleToGroundTruth(std::vector<FrameMatchLocation> &gt_other );


#endif // DFC_UTILS_H
