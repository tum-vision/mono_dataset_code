#include "aruco_cvversioning.h"
#include "dcf_utils.h"
#include "bitset"
#include "iostream"

void cornerClassifier(const cv::Mat &im, std::vector<cv::Point2f> points,std::vector<int>& corner_class, int wsize)
{
    corner_class.resize(points.size());

    if(im.type()!=CV_8UC1)
        throw std::runtime_error("assignClass Input image must be 8UC1");
    int wsizeFull=wsize*2+1;

    cv::Mat labels = cv::Mat::zeros(wsize*2+1,wsize*2+1,CV_8UC1);
    cv::Mat thresIm=cv::Mat(wsize*2+1,wsize*2+1,CV_8UC1);

    for(unsigned int i=0; i<4; i++)
    {
        float x = points[i].x;
        float y = points[i].y;

        x= int(x+0.5f);
        y= int(y+0.5f);

        cv::Rect r= cv::Rect(x-wsize,y-wsize,wsize*2+1,wsize*2+1);
        //Check boundaries
        if(r.x<0 || r.x+r.width>im.cols || r.y<0 || r.y+r.height>im.rows){
            corner_class[i]=-1;
            continue;
        }

        int endX=r.x+r.width;
        int endY=r.y+r.height;
        uchar minV=255,maxV=0;
        for(int y=r.y; y<endY; y++){
            const uchar *ptr=im.ptr<uchar>(y);
            for(int x=r.x; x<endX; x++)
            {
                if(minV>ptr[x]) minV=ptr[x];
                if(maxV<ptr[x]) maxV=ptr[x];
            }
        }

        if ((maxV-minV) < 25) {
            corner_class[i] = -1;
            continue;
        }

        double thres=(maxV+minV)/2.0;

        unsigned int nZ=0;
        //count non zero considering the threshold
        for(int y=0; y<wsizeFull; y++){
            const uchar *ptr=im.ptr<uchar>( r.y+y)+r.x;
            uchar *thresPtr= thresIm.ptr<uchar>(y);
            for(int x=0; x<wsizeFull; x++){
                if( ptr[x]>thres) {
                    nZ++;
                    thresPtr[x]=255;
                }
                else thresPtr[x]=0;
            }
        }
        //set all to zero labels.setTo(cv::Scalar::all(0));
        for(int y=0; y<thresIm.rows; y++){
            uchar *labelsPtr=labels.ptr<uchar>(y);
            for(int x=0; x<thresIm.cols; x++) labelsPtr[x]=0;
        }

        uchar newLab = 1;
        std::map<uchar, uchar> unions;
        for(int y=0; y<thresIm.rows; y++){
            uchar *thresPtr=thresIm.ptr<uchar>(y);
            uchar *labelsPtr=labels.ptr<uchar>(y);
            for(int x=0; x<thresIm.cols; x++)
            {
                uchar reg = thresPtr[x];
                uchar lleft_px = 0;
                uchar ltop_px = 0;

                if(x-1 > -1)
                {
                    if(reg == thresPtr[x-1])
                        lleft_px =labelsPtr[x-1];
                }

                if(y-1 > -1)
                {
                    if(reg ==thresIm.ptr<uchar>(y-1) [x]
                            )//thresIm.at<uchar>(y-1, x)
                        ltop_px =  labels.at<uchar>(y-1, x);
                }

                if(lleft_px==0 && ltop_px==0)
                    labelsPtr[x] = newLab++;

                else if(lleft_px!=0 && ltop_px!=0)
                {
                    if(lleft_px < ltop_px)
                    {
                        labelsPtr[x]  = lleft_px;
                        unions[ltop_px] = lleft_px;
                    }
                    else if(lleft_px > ltop_px)
                    {
                        labelsPtr[x]  = ltop_px;
                        unions[lleft_px] = ltop_px;
                    }
                    else
                    {//IGuales
                        labelsPtr[x]  = ltop_px;
                    }
                }
                else
                {
                    if(lleft_px!=0) labelsPtr[x]  = lleft_px;
                    else labelsPtr[x]  = ltop_px;
                }
            }
        }

        int nc= newLab-1 - unions.size();
        if(nc==2)
        {
            if(nZ > thresIm.total()-nZ) corner_class[i] = 0;
            else corner_class[i] = 1;
        }
        else if (nc > 2) {
            corner_class[i] = 2;
        }
    }
}

cv::Rect getRectFrom4Points(std::vector<cv::Point2f>points, cv::Size imSize, bool imFit)
{
    int minX=std::numeric_limits<int>::max(), maxX=std::numeric_limits<int>::min(),
            minY=std::numeric_limits<int>::max(), maxY=std::numeric_limits<int>::min();

    for(auto pt:points)
    {
        if(pt.x<minX) minX=pt.x;
        if(pt.y<minY) minY=pt.y;
        if(pt.x>maxX) maxX=pt.x;
        if(pt.y>maxY) maxY=pt.y;
    }

    if(!imSize.empty())
    {
        //The rect is out image
        if(minX > imSize.width || minY > imSize.height || maxX < 0 || maxY <0)
            return cv::Rect2d();

        if(imFit)
        {
            if(minX<0) minX=0;
            if(minY<0) minY=0;
            if(maxX>imSize.width) maxX=imSize.width;
            if(maxY>imSize.height) maxY=imSize.height;
        }
        else if(minX<0 || minY<0 || maxX>imSize.width || maxY>imSize.height)
            return cv::Rect2d();
    }

    return cv::Rect2d(cv::Point2d(minX,minY),cv::Point2d(maxX,maxY));
}

bool getMarkerIds(const cv::Mat& im, int total_bits, std::vector<cv::Point2f>& marker_corners, std::vector<uint64_t>& ids)
{
    int threshold=100;//125;
    const int nBitsSquared = static_cast<int>(std::sqrt(total_bits))+2;

    std::vector<cv::Point2f> obj_corners;
    obj_corners.push_back(cv::Point2f(0, 0));
    obj_corners.push_back(cv::Point2f(nBitsSquared, 0));
    obj_corners.push_back(cv::Point2f(nBitsSquared, nBitsSquared));
    obj_corners.push_back(cv::Point2f(0, nBitsSquared));

    ///Compute homography using marker corners
    cv::Mat H = cv::findHomography(obj_corners, marker_corners, CV_RANSAC);
    if(H.empty())
        return false;

    //Use homography to get the centers bit from marker
    std::vector<cv::Point2f> src_bit_centers;
    for(float y=1.5; y<nBitsSquared-1; y++)
        for(float x=1.5; x<nBitsSquared-1; x++)
            src_bit_centers.push_back(cv::Point2f(x,y));

    std::vector<cv::Point2f> vcenter_px;
    cv::perspectiveTransform(src_bit_centers, vcenter_px, H);

    int bits_noborder = static_cast<int>(std::sqrt(total_bits));

    //take the inner code
    cv::Mat _bits(bits_noborder,bits_noborder,CV_8UC1);
    for(int y=0;y<bits_noborder;y++)
        for(int x=0;x<bits_noborder;x++)
        {
            if(vcenter_px[y*bits_noborder+x].x+0.5f>im.cols || vcenter_px[y*bits_noborder+x].x-0.5f<0 ||
                vcenter_px[y*bits_noborder+x].y+0.5f>im.rows || vcenter_px[y*bits_noborder+x].y-0.5f<0)
                    return false;


            if(getSubpixelValue(im, vcenter_px[y*bits_noborder+x])< threshold)
                _bits.at<uchar>(y,x)=0;
            else
                _bits.at<uchar>(y,x)=1;
        }

    // now, get the 64bits ids
    int nr = 0;
    do
    {
        ids.push_back(touulong(_bits));
        _bits = rotate(_bits);
        nr++;
    } while (nr < 4);

    return true;
}

/**Gets the subpixel value of the point passed
 * @param im_grey image
 * @param p point2d
 */
float getSubpixelValue(const cv::Mat &im_grey,const cv::Point2f &p){

     assert(im_grey.type()==CV_8UC1);
     float intpartX;
     float decpartX=std::modf(p.x,&intpartX);
     float intpartY;
     float decpartY=std::modf(p.y,&intpartY);

     cv::Point tl;

     if (decpartX>0.5) {
         if (decpartY>0.5) tl=cv::Point(intpartX,intpartY);
         else tl=cv::Point(intpartX,intpartY-1);
     }
     else{
         if (decpartY>0.5) tl=cv::Point(intpartX-1,intpartY);
         else tl=cv::Point(intpartX-1,intpartY-1);
     }
     return (1.f-decpartY)*(1.-decpartX)*float(im_grey.at<uchar>(tl.y,tl.x))+
        decpartX*(1-decpartY)*float(im_grey.at<uchar>(tl.y,tl.x+1))+
        (1-decpartX)*decpartY*float(im_grey.at<uchar>(tl.y+1,tl.x))+
        decpartX*decpartY*float(im_grey.at<uchar>(tl.y+1,tl.x+1));
}

// convert matrix of (0,1)s in a 64 bit value
uint64_t touulong(const cv::Mat& code)
{
    std::bitset<64> bits;
    int bidx = 0;
    for (int y = code.rows - 1; y >= 0; y--)
        for (int x = code.cols - 1; x >= 0; x--)
            bits[bidx++] = code.at<uchar>(y, x);
    return bits.to_ullong();
}

//Rotate matrix
cv::Mat rotate(const cv::Mat& in)
{
    cv::Mat out;
    in.copyTo(out);
    for (int i = 0; i < in.rows; i++)
    {
        for (int j = 0; j < in.cols; j++)
        {
            out.at<uchar>(i, j) = in.at<uchar>(in.cols - j - 1, i);
        }
    }
    return out;
}


cv::Mat  alignAndScaleToGroundTruth(std::vector<FrameMatchLocation> &gt_other ){

    std::vector<cv::Point3d> points_other,points_gt;
    for(auto gto:gt_other){
        assert(gto.first.type()==CV_64F);
        points_gt.push_back(cv::Point3d(gto.first.at<double>(0,3),gto.first.at<double>(1,3),gto.first.at<double>(2,3)));
        points_other.push_back(cv::Point3d(gto.second.at<double>(0,3),gto.second.at<double>(1,3),gto.second.at<double>(2,3)));
    }

    cv::Mat best_T= rigidBodyTransformation_Horn1987(points_other,points_gt,false);
     cv::Mat best_T64;
    if( best_T.type()!=CV_64F) best_T.convertTo(best_T64,CV_64F);
    else best_T64=best_T;
    //cout<<best_T64<<endl;
    for(auto &gto:gt_other){
         gto.second = best_T64* gto.second;
    }

    return best_T64;
}

int hamm_distance(uint64_t a, uint64_t b)
{
    return static_cast<int>(std::bitset<64>(a ^ b).count());
}


cv::Mat rigidBodyTransformation_Horn1987 (const std::vector<cv::Point3d> &org, const std::vector<cv::Point3d> &dst,bool mbFixScale){
    auto ComputeCentroid=[](cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
    {
        cv::reduce(P,C,1,CV_REDUCE_SUM);
        C = C/P.cols;
        for(int i=0; i<P.cols; i++)
            Pr.col(i)=P.col(i)-C;
    };

    // Custom implementation of:
    // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

    //Create the P1 an P2 matrices
    cv::Mat P1(3,org.size(),CV_32F);
    cv::Mat P2(3,org.size(),CV_32F);
    for(size_t i=0;i<org.size();i++){
        P1.at<float>(0,i)=org[i].x;
        P1.at<float>(1,i)=org[i].y;
        P1.at<float>(2,i)=org[i].z;
        P2.at<float>(0,i)=dst[i].x;
        P2.at<float>(1,i)=dst[i].y;
        P2.at<float>(2,i)=dst[i].z;
    }


        // Step 1: Centroid and relative coordinates

        cv::Mat Pr1(P1.size(),P1.type()); // Relative coordinates to centroid (set 1)
        cv::Mat Pr2(P2.size(),P2.type()); // Relative coordinates to centroid (set 2)
        cv::Mat O1(3,1,Pr1.type()); // Centroid of P1
        cv::Mat O2(3,1,Pr2.type()); // Centroid of P2

        ComputeCentroid(P1,Pr1,O1);
        ComputeCentroid(P2,Pr2,O2);

        // Step 2: Compute M matrix

        cv::Mat M = Pr2*Pr1.t();

        // Step 3: Compute N matrix

        double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

        cv::Mat N(4,4,P1.type());

        N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
        N12 = M.at<float>(1,2)-M.at<float>(2,1);
        N13 = M.at<float>(2,0)-M.at<float>(0,2);
        N14 = M.at<float>(0,1)-M.at<float>(1,0);
        N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
        N23 = M.at<float>(0,1)+M.at<float>(1,0);
        N24 = M.at<float>(2,0)+M.at<float>(0,2);
        N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
        N34 = M.at<float>(1,2)+M.at<float>(2,1);
        N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);

        N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                                     N12, N22, N23, N24,
                                     N13, N23, N33, N34,
                                     N14, N24, N34, N44);


        // Step 4: Eigenvector of the highest eigenvalue

        cv::Mat eval, evec;

        cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation

        cv::Mat vec(1,3,evec.type());
        (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

        // Rotation angle. sin is the norm of the imaginary part, cos is the real part
        double ang=atan2(norm(vec),evec.at<float>(0,0));

        if (norm(vec)<1e-7)return cv::Mat::eye(4,4,CV_32F);

        vec = 2*ang*vec/norm(vec); //Angle-axis representation. quaternion angle is the half

        cv::Mat mR12i(3,3,P1.type());

        cv::Rodrigues(vec,mR12i); // computes the rotation matrix from angle-axis

        // Step 5: Rotate set 2

        cv::Mat P3 = mR12i*Pr2;

        // Step 6: Scale
        float ms12i;

        if(!mbFixScale)
        {
            double nom = Pr1.dot(P3);
            cv::Mat aux_P3(P3.size(),P3.type());
            aux_P3=P3;
            cv::pow(P3,2,aux_P3);
            double den = 0;

            for(int i=0; i<aux_P3.rows; i++)
            {
                for(int j=0; j<aux_P3.cols; j++)
                {
                    den+=aux_P3.at<float>(i,j);
                }
            }

            ms12i = nom/den;
        }
        else
            ms12i = 1.0f;

        // Step 7: Translation

        cv::Mat  mt12i(1,3,P1.type());
        mt12i = O1 - ms12i*mR12i*O2;

        // Step 8: Transformation

        // Step 8.1 T12
        cv::Mat mT12i = cv::Mat::eye(4,4,P1.type());

        cv::Mat sR = ms12i*mR12i;

        sR.copyTo(mT12i.rowRange(0,3).colRange(0,3));
        mt12i.copyTo(mT12i.rowRange(0,3).col(3));
//        return mT12i;

//        // Step 8.2 T21

        cv::Mat mT21i = cv::Mat::eye(4,4,P1.type());

        cv::Mat sRinv = (1.0/ms12i)*mR12i.t();

        sRinv.copyTo(mT21i.rowRange(0,3).colRange(0,3));
        cv::Mat tinv = -sRinv*mt12i;
        tinv.copyTo(mT21i.rowRange(0,3).col(3));
        return mT21i;
}


