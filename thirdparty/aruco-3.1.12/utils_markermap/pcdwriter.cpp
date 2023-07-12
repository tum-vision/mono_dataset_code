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

#include "markermap.h"
#include <fstream>
#include <map>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace std;

void getRTfromMatrix44(const cv::Mat& M, cv::Mat& R, cv::Mat& T)
{
    assert(M.cols == M.rows && M.cols == 4);
    assert(M.type() == CV_32F || M.type() == CV_64F);
    // extract the rotation part
    cv::Mat r33 = cv::Mat(M, cv::Rect(0, 0, 3, 3));
    cv::SVD svd(r33);
    cv::Mat Rpure = svd.u * svd.vt;
    cv::Rodrigues(Rpure, R);
    T.create(1, 3, M.type());
    if (M.type() == CV_32F)
        for (int i = 0; i < 3; i++)
            T.ptr<float>(0)[i] = M.at<float>(i, 3);
    else
        for (int i = 0; i < 3; i++)
            T.ptr<double>(0)[i] = M.at<double>(i, 3);
}

/**********************
 *
 *
 **********************/
struct Quaternion
{
    Quaternion(float q0, float q1, float q2, float q3)
    {
        q[0] = q0;
        q[1] = q1;
        q[2] = q2;
        q[3] = q3;
    }
    cv::Mat getRotation() const
    {
        cv::Mat R(3, 3, CV_32F);
        R.at<float>(0, 0) = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
        R.at<float>(0, 1) = 2.f * (q[1] * q[2] - q[0] * q[3]);
        R.at<float>(0, 2) = 2.f * (q[1] * q[3] + q[0] * q[2]);

        R.at<float>(1, 0) = 2.f * (q[1] * q[2] + q[0] * q[3]);
        R.at<float>(1, 1) = q[0] * q[0] + q[2] * q[2] - q[1] * q[1] - q[3] * q[3];
        R.at<float>(1, 2) = 2.f * (q[2] * q[3] - q[0] * q[1]);

        R.at<float>(2, 0) = 2.f * (q[1] * q[3] - q[0] * q[2]);
        R.at<float>(2, 1) = 2.f * (q[2] * q[3] + q[0] * q[1]);
        R.at<float>(2, 2) = q[0] * q[0] + q[3] * q[3] - q[1] * q[1] - q[2] * q[2];
        return R;
    }
    float q[4];
};

/**********************
 *
 *
 **********************/
float rigidBodyTransformation_Horn1987(cv::Mat& S, cv::Mat& M, cv::Mat& RT_4x4)
{
    assert(S.total() == M.total());
    assert(S.type() == M.type());
    assert(S.rows > S.cols && M.rows > M.cols);

    cv::Mat _s, _m;
    S.convertTo(_s, CV_32F);
    M.convertTo(_m, CV_32F);
    _s = _s.reshape(1);
    _m = _m.reshape(1);
    cv::Mat Mu_s = cv::Mat::zeros(1, 3, CV_32F);
    cv::Mat Mu_m = cv::Mat::zeros(1, 3, CV_32F);
    //         cout<<_s<<endl<<_m<<endl;
    // calculate means
    for (int i = 0; i < _s.rows; i++)
    {
        Mu_s += _s(cv::Range(i, i + 1), cv::Range(0, 3));
        Mu_m += _m(cv::Range(i, i + 1), cv::Range(0, 3));
    }
    // now, divide
    for (int i = 0; i < 3; i++)
    {
        Mu_s.ptr<float>(0)[i] /= float(_s.rows);
        Mu_m.ptr<float>(0)[i] /= float(_m.rows);
    }

    // cout<<"Mu_s="<<Mu_s<<endl;
    // cout<<"Mu_m="<<Mu_m<<endl;

    cv::Mat Mu_st = Mu_s.t() * Mu_m;
    // cout<<"Mu_st="<<Mu_st<<endl;
    cv::Mat Var_sm = cv::Mat::zeros(3, 3, CV_32F);
    for (int i = 0; i < _s.rows; i++)
        Var_sm += (_s(cv::Range(i, i + 1), cv::Range(0, 3)).t() * _m(cv::Range(i, i + 1), cv::Range(0, 3))) - Mu_st;
    //   cout<<"Var_sm="<<Var_sm<<endl;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Var_sm.at<float>(i, j) /= float(_s.rows);
    //   cout<<"Var_sm="<<Var_sm<<endl;

    cv::Mat AA = Var_sm - Var_sm.t();
    //     cout<<"AA="<<AA<<endl;
    cv::Mat A(3, 1, CV_32F);
    A.at<float>(0, 0) = AA.at<float>(1, 2);
    A.at<float>(1, 0) = AA.at<float>(2, 0);
    A.at<float>(2, 0) = AA.at<float>(0, 1);
    //     cout<<"A ="<<A <<endl;
    cv::Mat Q_Var_sm(4, 4, CV_32F);
    Q_Var_sm.at<float>(0, 0) = static_cast<float>(trace(Var_sm)[0]);
    for (int i = 1; i < 4; i++)
    {
        Q_Var_sm.at<float>(0, i) = A.ptr<float>(0)[i - 1];
        Q_Var_sm.at<float>(i, 0) = A.ptr<float>(0)[i - 1];
    }
    cv::Mat q33 = Var_sm + Var_sm.t() - (trace(Var_sm)[0] * cv::Mat::eye(3, 3, CV_32F));

    cv::Mat Q33 = Q_Var_sm(cv::Range(1, 4), cv::Range(1, 4));
    q33.copyTo(Q33);
    // cout<<"Q_Var_sm"<<endl<< Q_Var_sm<<endl;
    cv::Mat eigenvalues, eigenvectors;
    eigen(Q_Var_sm, eigenvalues, eigenvectors);
    // cout<<"EEI="<<eigenvalues<<endl;
    // cout<<"V="<<(eigenvectors.type()==CV_32F)<<" "<<eigenvectors<<endl;

    Quaternion rot(eigenvectors.at<float>(0, 0), eigenvectors.at<float>(0, 1), eigenvectors.at<float>(0, 2),
                   eigenvectors.at<float>(0, 3));
    cv::Mat RR = rot.getRotation();
    //  cout<<"RESULT="<<endl<<RR<<endl;
    cv::Mat T = Mu_m.t() - RR * Mu_s.t();
    //  cout<<"T="<<T<<endl;

    RT_4x4 = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat r33 = RT_4x4(cv::Range(0, 3), cv::Range(0, 3));
    RR.copyTo(r33);
    for (int i = 0; i < 3; i++)
        RT_4x4.at<float>(i, 3) = T.ptr<float>(0)[i];
    //  cout<<"RESS="<<RT<<endl;

    // compute the average transform error

    float err = 0;
    float* matrix = RT_4x4.ptr<float>(0);
    for (int i = 0; i < S.rows; i++)
    {
        cv::Point3f org = S.ptr<cv::Point3f>(0)[i];
        cv::Point3f dest_est;
        dest_est.x = matrix[0] * org.x + matrix[1] * org.y + matrix[2] * org.z + matrix[3];
        dest_est.y = matrix[4] * org.x + matrix[5] * org.y + matrix[6] * org.z + matrix[7];
        dest_est.z = matrix[8] * org.x + matrix[9] * org.y + matrix[10] * org.z + matrix[11];
        cv::Point3f dest_real = M.ptr<cv::Point3f>(0)[i];
        err += static_cast<float>(cv::norm(dest_est - dest_real));
    }
    return err / float(S.rows);
    ;
}
/**********************
 *
 *
 **********************/
float rigidBodyTransformation_Horn1987(cv::Mat& _s, cv::Mat& _m, cv::Mat& Rvec, cv::Mat& Tvec)
{
    cv::Mat RT;
    float err = rigidBodyTransformation_Horn1987(_s, _m, RT);
    getRTfromMatrix44(RT, Rvec, Tvec);
    return err;
}

float rigidBodyTransformation_Horn1987(const vector<cv::Point3f>& orgPoints_32FC3,
                                       const vector<cv::Point3f>& dstPoints_32FC3, cv::Mat& Rvec, cv::Mat& Tvec)
{
    cv::Mat Morg, Mdest;
    Morg.create(static_cast<int>(orgPoints_32FC3.size()), 1, CV_32FC3);
    Mdest.create(static_cast<int>(dstPoints_32FC3.size()), 1, CV_32FC3);
    for (size_t i = 0; i < dstPoints_32FC3.size(); i++)
    {
        Morg.ptr<cv::Point3f>(0)[i] = orgPoints_32FC3[i];
        Mdest.ptr<cv::Point3f>(0)[i] = dstPoints_32FC3[i];
    }
    return rigidBodyTransformation_Horn1987(Morg, Mdest, Rvec, Tvec);
}

cv::Mat getRTMatrix(const cv::Mat& R_, const cv::Mat& T_, int forceType = -1)
{
    cv::Mat M;
    cv::Mat R, T;
    R_.copyTo(R);
    T_.copyTo(T);
    if (R.type() == CV_64F)
    {
        assert(T.type() == CV_64F);
        cv::Mat Matrix = cv::Mat::eye(4, 4, CV_64FC1);

        cv::Mat R33 = cv::Mat(Matrix, cv::Rect(0, 0, 3, 3));
        if (R.total() == 3)
        {
            cv::Rodrigues(R, R33);
        }
        else if (R.total() == 9)
        {
            cv::Mat R64;
            R.convertTo(R64, CV_64F);
            R.copyTo(R33);
        }
        for (int i = 0; i < 3; i++)
            Matrix.at<double>(i, 3) = T.ptr<double>(0)[i];
        M = Matrix;
    }
    else if (R.depth() == CV_32F)
    {
        cv::Mat Matrix = cv::Mat::eye(4, 4, CV_32FC1);
        cv::Mat R33 = cv::Mat(Matrix, cv::Rect(0, 0, 3, 3));
        if (R.total() == 3)
        {
            cv::Rodrigues(R, R33);
        }
        else if (R.total() == 9)
        {
            cv::Mat R32;
            R.convertTo(R32, CV_32F);
            R.copyTo(R33);
        }

        for (int i = 0; i < 3; i++)
            Matrix.at<float>(i, 3) = T.ptr<float>(0)[i];
        M = Matrix;
    }

    if (forceType == -1)
        return M;
    else
    {
        cv::Mat MTyped;
        M.convertTo(MTyped, forceType);
        return MTyped;
    }
}

cv::Mat rigidBodyTransformation_Horn1987(const std::vector<cv::Point3f>& org, const std::vector<cv::Point3f>& dst,
                                         double* err = 0)
{
    double e = 0;
    cv::Mat r, t;
    e = rigidBodyTransformation_Horn1987(org, dst, r, t);
    if (err)
        *err = e;
    return getRTMatrix(r, t);
}

cv::Point3f mult(const cv::Mat& m, cv::Point3f p)
{
    assert(m.isContinuous());
    cv::Point3f res;
    if (m.type() == CV_32F)
    {
        const float* ptr = m.ptr<float>(0);
        res.x = ptr[0] * p.x + ptr[1] * p.y + ptr[2] * p.z + ptr[3];
        res.y = ptr[4] * p.x + ptr[5] * p.y + ptr[6] * p.z + ptr[7];
        res.z = ptr[8] * p.x + ptr[9] * p.y + ptr[10] * p.z + ptr[11];
    }
    else if (m.type() == CV_64F)
    {
        const double* ptr = m.ptr<double>(0);
        cv::Point3f res;
        res.x = static_cast<float>(ptr[0] * p.x + ptr[1] * p.y + ptr[2] * p.z + ptr[3]);
        res.y = static_cast<float>(ptr[4] * p.x + ptr[5] * p.y + ptr[6] * p.z + ptr[7]);
        res.z = static_cast<float>(ptr[8] * p.x + ptr[9] * p.y + ptr[10] * p.z + ptr[11]);
    }
    return res;
}

std::vector<cv::Vec4f> getPcdPoints(const vector<cv::Point3f>& mpoints, cv::Scalar color, int npoints = 100)
{
    vector<cv::Vec4f> points;
    double msize = cv::norm(mpoints[0] - mpoints[1]);
    float fcolor;
    uchar* c = (uchar*)&fcolor;
    for (int i = 0; i < 3; i++)
        c[i] = static_cast<uchar>(color[i]);

    // lines joining points
    for (size_t i = 0; i < mpoints.size(); i++)
    {
        cv::Point3f v = mpoints[(i + 1) % mpoints.size()] - mpoints[i];
        float ax = 1.f / float(npoints);  // npoints
        for (float x = 0; x <= 1; x += ax)
        {
            cv::Point3f p3 = mpoints[i] + v * x;
            points.push_back(cv::Vec4f(p3.x, p3.y, p3.z, fcolor));
        }
    }

    // line indicating direction
    // take first and second, first and last , and get the cross vector indicating the direction
    cv::Point3f v1 = mpoints[1] - mpoints[0];
    cv::Point3f v2 = mpoints[3] - mpoints[0];
    v1 *= 1. / cv::norm(v1);
    v2 *= 1. / cv::norm(v2);
    cv::Point3f vz = v2.cross(v1);
    vz *= 1. / cv::norm(vz);  // now, unit

    // set the center
    cv::Point3f center = (mpoints[0] + mpoints[1] + mpoints[2] + mpoints[3]) * 0.25;
    const float ax = static_cast<float>((msize / 3.) / 100.);
    for (float x = 0; x <= msize / 3; x += ax)
    {
        cv::Point3f p3 = center + vz * x;
        points.push_back(cv::Vec4f(p3.x, p3.y, p3.z, fcolor));
    }

    return points;
}

vector<cv::Vec4f> getMarkerIdPcd(aruco::Marker3DInfo& minfo, cv::Scalar color)
{
    int id = minfo.id;
    float markerSize = minfo.getMarkerSize();
    cv::Mat rt_g2m = rigidBodyTransformation_Horn1987(aruco::Marker::get3DPoints(markerSize), minfo.points);
    // marker id as a set of points
    string text = std::to_string(id);
    int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 2;
    int thickness = 3;
    int baseline = 0;
    float markerSize_2 = markerSize / 2;
    cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
    cv::Mat img(textSize + cv::Size(0, baseline / 2), CV_8UC1, cv::Scalar::all(0));
    // center the text
    // then put the text itself
    cv::putText(img, text, cv::Point(0, textSize.height + baseline / 4), fontFace, fontScale, cv::Scalar::all(255),
                thickness, 8);
    // raster 2d points as 3d points
    vector<cv::Point3f> points_id;
    for (int y = 0; y < img.rows; y++)
        for (int x = 0; x < img.cols; x++)
            if (img.at<uchar>(y, x) != 0)
                points_id.push_back(
                    cv::Point3f((float(x) / float(img.cols)) - 0.5f, (float(img.rows - y) / float(img.rows)) - 0.5f, 0.f));

    // now,scale
    for (auto& p : points_id)
        p *= markerSize_2;
    // finally, translate
    for (auto& p : points_id)
        p = mult(rt_g2m, p);
    // now, add to ouput

    float fcolor;
    uchar* c = (uchar*)&fcolor;
    for (int i = 0; i < 3; i++)
        c[i] = static_cast<uchar>(color[i]);

    vector<cv::Vec4f> res;
    for (auto& p : points_id)
        res.push_back(cv::Vec4f(p.x, p.y, p.z, fcolor));

    return res;
}

void savePCDFile(string fpath, const aruco::MarkerMap& ms,
                 const std::map<int, cv::Mat> frame_pose_map)
{
    std::vector<cv::Vec4f> points2write;
    for (auto m : ms)
    {
        cv::Scalar color;
        color = cv::Scalar(255, 0, 0);
        auto points4 = getPcdPoints(m.points, color);
        points2write.insert(points2write.end(), points4.begin(), points4.end());
        auto points_id = getMarkerIdPcd(m, color);
        points2write.insert(points2write.end(), points_id.begin(), points_id.end());
        // max_msize=std::max(max_msize,m.second.markerSize);
    }
    for (auto frame_pose : frame_pose_map)
    {
        if (!frame_pose.second.empty())
        {
            cv::Mat g2c = frame_pose.second.inv();
            auto mpoints = aruco::Marker::get3DPoints(ms[0].getMarkerSize() / 2);
            for (auto& p : mpoints)
                p = mult(g2c, p);
            auto pcam = getPcdPoints(mpoints, cv::Scalar(0, 255, 0), 25);
            points2write.insert(points2write.end(), pcam.begin(), pcam.end());
        }
    }
    // now, the 3d points

    std::ofstream filePCD(fpath, std::ios::binary);

    filePCD << "# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F "
               "F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "
            << points2write.size() << "\nHEIGHT 1\nPOINTS " << points2write.size() << "\nDATA binary\n";

    filePCD.write((char*)&points2write[0], points2write.size() * sizeof(points2write[0]));
}
