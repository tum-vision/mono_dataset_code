#include "trackerimpl.h"
#include "bitset"
#include "opencv2/calib3d/calib3d.hpp"
#include "dcf_utils.h"

TrackerImpl::TrackerImpl(const Marker& marker, uint64_t code, uint32_t dic_bits, const cv::Mat& image, double psr, int tau_s)
{
    psrThreshold = psr;
    markerWarpSize = tau_s;
    cornerPatchSize = cv::Size(tau_s, tau_s);

    TheMarkerCode = code;
    nbits = dic_bits;
    TheMarker = marker;
    prepare(marker, image);
}


TrackerImpl::~TrackerImpl()
{

}


void TrackerImpl::prepare(const Marker& marker,const cv::Mat& image)
{
    int     minX=std::numeric_limits<int>::max(), maxX=std::numeric_limits<int>::min(),
            minY=std::numeric_limits<int>::max(), maxY=std::numeric_limits<int>::min();

    for(auto pt:marker)
    {
        if(pt.x<minX) minX=pt.x;
        if(pt.y<minY) minY=pt.y;
        if(pt.x>maxX) maxX=pt.x;
        if(pt.y>maxY) maxY=pt.y;

        center.x += pt.x;
        center.y += pt.y;
    }

    boundingBox = cv::Rect2d(cv::Point2d(minX,minY),cv::Point2d(maxX,maxY));
    center.x /= marker.size();
    center.y /= marker.size();

    double desiredarea = std::pow(markerWarpSize, 2.f);
    double boundingBoxArea = boundingBox.area();
    double pyrfactor2 = pow(pyrfactor, 2);

    int p = 1;
    while (boundingBoxArea / pow(pyrfactor2, p) >= desiredarea)
        imgPyrIdx = p++;

    int s = int(floor(markerWarpSize * (1+padding)));
    patchSize = cv::Size(s, s);
    if(contextAware)
        s = int(floor(2*markerWarpSize + s));
    roiSize = cv::Size(s, s);

    int realRoiSize = int(floor(s * pow(pyrfactor, imgPyrIdx)));
    cv::getRectSubPix(image, cv::Size(realRoiSize, realRoiSize), center, roi);
    cv::resize(roi,roi,roiSize);

//    cv::imshow("roi", roi);
//    cv::waitKey();
}

bool TrackerImpl::initImpl()
{
    if(roi.empty())
        return false;

    ScopedTimerEvents Timer("TrackerImpl::initImpl");

    createHanningWindow(hanWin, patchSize, CV_32F);

    // goal
    cv::Mat g=cv::Mat::zeros(patchSize, CV_32F);
    g.at<float>(patchSize.height/2, patchSize.width/2) = 1;
    GaussianBlur(g, g, cv::Size(-1,-1), 2.0);
    double maxVal;
    minMaxLoc(g, 0, &maxVal);
    g = g / maxVal;
    dft(g, G, cv::DFT_COMPLEX_OUTPUT);

    cv::Mat patch;
    //Get the center position
    int x = int(floor((roi.cols)/2));
    int y = int(floor((roi.rows)/2));
    cv::Point2f patchCenter(x,y);
    getRectSubPix(roi, patchSize, patchCenter, patch);

    // initial A,B and H
    A = cv::Mat::zeros(G.size(), G.type());
    B = cv::Mat::zeros(G.size(), G.type());
    C = cv::Mat::zeros(G.size(), G.type());
    cv::Mat WINDOW_WARP, A_i, B_i,C_i;
    for(int i=0; i<8; i++)
    {
        cv::Mat window_warp = randWarp(patch);
        preProcess(window_warp, hanWin);
        dft(window_warp, WINDOW_WARP, cv::DFT_COMPLEX_OUTPUT);
        mulSpectrums(G          , WINDOW_WARP, A_i, 0, true);
        mulSpectrums(WINDOW_WARP, WINDOW_WARP, B_i, 0, true);
        A+=A_i;
        B+=B_i;
    }

    //now the context
    if(contextAware){
        for(int i=0;i<4;i++){
            auto center2=center/pow(pyrfactor, imgPyrIdx);
            if( i==0) center2.x+=patchSize.width;
            else if(i==1)center2.x-=patchSize.width;
            else if( i==2) center2.y+=patchSize.height;
            else if(i==3)center2.y-=patchSize.height;

            cv::Mat sub_win;
            getRectSubPix(roi, patchSize, center2, sub_win); preProcess(sub_win, hanWin);
            dft(sub_win, WINDOW_WARP, cv::DFT_COMPLEX_OUTPUT);
            mulSpectrums(WINDOW_WARP, WINDOW_WARP, C_i, 0, true);
            C+=C_i;
        }
        H = divDFTs(A,lambda1+B+lambda2*C);
    }
    else
        H = divDFTs(A,lambda1+B);


    //Corners!!
    createHanningWindow(cornerhanWin, cornerPatchSize, CV_32F);
    // goal
    cv::Mat gc=cv::Mat::zeros(cornerPatchSize,CV_32F);
    gc.at<float>(cornerPatchSize.height/2, cornerPatchSize.width/2) = 1;
    GaussianBlur(gc, gc, cv::Size(-1,-1), 2.0);
    double maxValc;
    minMaxLoc(gc, 0, &maxValc);
    gc = gc / maxValc;
    dft(gc, Gc, cv::DFT_COMPLEX_OUTPUT);

    // initial corners A,B and H
    for(size_t i=0;i<4;i++)
    {
        Ac.push_back(cv::Mat::zeros(Gc.size(), Gc.type()));
        Bc.push_back(cv::Mat::zeros(Gc.size(), Gc.type()));

        cv::Mat sub_win;
        cv::Point2d p = patchCenter - (center - TheMarker[i])/pow(pyrfactor, imgPyrIdx);
        getRectSubPix(roi, cornerPatchSize, p, sub_win);
//        cv::imshow("Roi",roi);
//        cv::waitKey();
//        cv::imshow("Patch corner",sub_win);
//        cv::waitKey();
        preProcess(sub_win, cornerhanWin);
        dft(sub_win, WINDOW_WARP, cv::DFT_COMPLEX_OUTPUT);
        mulSpectrums(Gc          , WINDOW_WARP, A_i, 0, true);
        mulSpectrums(WINDOW_WARP, WINDOW_WARP, B_i, 0, true);
        Ac[i]+=A_i;
        Bc[i]+=B_i;
        Hc.push_back(divDFTs(Ac[i],lambda1+Bc[i]));
    }

    return true;
}

bool TrackerImpl::updateImpl(const cv::Mat& image, float thrs_confidence)
{
    //If H empty try init
    if(H.empty())
        if(!initImpl())
            return false;

//    std::cout << "### Update Marker ID : "<< TheMarker.id<<std::endl;
//    std::cout << "Pyramid index : "<< imgPyrIdx<<std::endl;

    trust=0;
    //////
    /// Optimal scale
    ///
    ScopedTimerEvents Timer("TrackerImpl::updateImpl");
    cv::Point2f roiCenter;
    if(!optimalScale(image, roiCenter))
        return false;
    Timer.add("optimalScale");

    cv::Mat img_sub_new;
    if(track_corners)
    {
        int goodCorners=0;
        //Update corners
        for(size_t i=0;i<4;i++)
        {
            cv::Mat img_sub_new;
            cv::Point2f cornerOffset = roiCenter + ((TheMarker[i] - center)/pow(pyrfactor, imgPyrIdx));
            getRectSubPix(roi, cornerPatchSize, cornerOffset, img_sub_new);
            if (img_sub_new.channels() != 1)
                cvtColor(img_sub_new, img_sub_new, cv::COLOR_BGR2GRAY);
//            cv::imshow("xx", img_sub_new);
//            cv::waitKey();

            preProcess(img_sub_new, cornerhanWin);

            cv::Point delta_xy;
            float c=correlate(img_sub_new, delta_xy, Hc[i]);
            if(c > psrThreshold)
            {
                goodCorners++;
                TheMarker[i].x += delta_xy.x*pow(pyrfactor, imgPyrIdx);
                TheMarker[i].y += delta_xy.y*pow(pyrfactor, imgPyrIdx);
            }
        }

        if(goodCorners == 4)
        {
            buildCornersPyramid(image);
            cornerUpsample(TheMarker);

            for(size_t i=0;i<4;i++)
            {
                cv::Point2f cornerOffset = roiCenter + ((TheMarker[i] - center)/pow(pyrfactor, imgPyrIdx));
                getRectSubPix(roi, cornerPatchSize, cornerOffset, img_sub_new);
                preProcess(img_sub_new, cornerhanWin);

                // new state for A and B
                cv::Mat F, A_new, B_new,C_new;
                C_new=cv::Mat::zeros(Gc.size(), Gc.type());

                dft(img_sub_new, F, cv::DFT_COMPLEX_OUTPUT);
                mulSpectrums(Gc, F, A_new, 0, true );
                mulSpectrums(F, F, B_new, 0, true );

                // update A ,B, and H
                Ac[i] = Ac[i]*(1-rate) + A_new*rate;
                Bc[i] = Bc[i]*(1-rate) + B_new*rate;
                Hc[i] = divDFTs(Ac[i], lambda1+Bc[i]);
            }
        }

        float minX=std::numeric_limits<float>::max(), maxX=std::numeric_limits<float>::min(),
                minY=std::numeric_limits<float>::max(), maxY=std::numeric_limits<float>::min();
        cv::Point2f newCenter(0,0);
        cv::Rect2f newBox;
        int hamm_dst = nbits;


        for(size_t i=0;i<4;i++)
        {
            if(TheMarker[i].x<minX) minX=TheMarker[i].x;
            if(TheMarker[i].y<minY) minY=TheMarker[i].y;
            if(TheMarker[i].x>maxX) maxX=TheMarker[i].x;
            if(TheMarker[i].y>maxY) maxY=TheMarker[i].y;
            newCenter.x+=TheMarker[i].x;
            newCenter.y+=TheMarker[i].y;
        }

        newBox = cv::Rect2f(cv::Point2f(minX,minY),cv::Point2f(maxX,maxY));
        if(goodCorners == 4)
        {
            center.x = newCenter.x/4;
            center.y = newCenter.y/4;

            double desiredarea = std::pow(markerWarpSize, 2.f);
            double pyrfactor2 = pow(pyrfactor,2);
            int p = 1;
            while (newBox.area() / pow(pyrfactor2, p) >= desiredarea)
                imgPyrIdx = p++;
        }

        //Check box edges out image
        if(newBox.tl().x < 0.f) newBox.x = 0.f;
        if(newBox.tl().y < 0.f) newBox.y = 0.f;
        if(newBox.br().x >= image.cols) newBox.width = image.cols - newBox.tl().x;
        if(newBox.br().y >= image.rows) newBox.height = image.rows - newBox.tl().y;
        //////
        //        cv::Mat out;
        //        cv::cvtColor(image, out, cv::COLOR_GRAY2BGR);
        //        for(auto pt:TheMarker)
        //        {
        //            std::cout << "PT:"<< pt << std::endl;
        //            cv::circle(out,pt,1,cv::Scalar(0,0,255),-1);
        //        }
        //        cv::rectangle(out,newBox,cv::Scalar(0,0,255));
        //        std::cout << "BOX:"<< newBox << std::endl;
        //        cv::imshow("Marker",out);
        //        cv::waitKey();
        ///////


        cv::Mat markerThres;
        cv::threshold(image(newBox), markerThres, 125, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        //Transform marker corners to subImage
        std::vector<cv::Point2f> im_corner;
        im_corner.push_back(cv::Point2f(TheMarker[0].x-newBox.tl().x, TheMarker[0].y-newBox.tl().y));
        im_corner.push_back(cv::Point2f(TheMarker[1].x-newBox.tl().x, TheMarker[1].y-newBox.tl().y));
        im_corner.push_back(cv::Point2f(TheMarker[2].x-newBox.tl().x, TheMarker[2].y-newBox.tl().y));
        im_corner.push_back(cv::Point2f(TheMarker[3].x-newBox.tl().x, TheMarker[3].y-newBox.tl().y));

        std::vector<uint64_t> rot_ids;
        if(!getMarkerIds(markerThres, nbits, im_corner, rot_ids))
            return false;


//        cv::Mat out;
//        cv::cvtColor(markerThres, out, cv::COLOR_GRAY2BGR);
//        for(auto pt:vpixels)
//            cv::circle(out,pt,1,cv::Scalar(0,0,255),-1);
//        cv::imshow("pxis"+std::to_string(TheMarker.id),out);
//                cv::waitKey();

        //Best distance in its 4 rotations
        for(auto id:rot_ids)
        {
            int tmp = hamm_distance(id, TheMarkerCode);
            if(tmp < hamm_dst)
                hamm_dst = tmp;
        }

        //Marker confidence
        trust = std::fabs(goodCorners * (hamm_dst/float(nbits) - 1.f)/4);
        if(trust < thrs_confidence)
            return false;
    }
    Timer.add("track-corners");


    getRectSubPix(roi, patchSize, roiCenter, img_sub_new);

//    cv::imshow("roi", roi);
//    cv::waitKey();
//    cv::imshow("pathc", warpmarker);
//    cv::waitKey();


    if (img_sub_new.channels() != 1)
        cvtColor(img_sub_new, img_sub_new, cv::COLOR_BGR2GRAY);
    preProcess(img_sub_new, hanWin);

    // new state for A and B
    cv::Mat F, A_new, B_new,C_new;
    C_new=cv::Mat::zeros(G.size(), G.type());

    dft(img_sub_new, F, cv::DFT_COMPLEX_OUTPUT);
    mulSpectrums(G, F, A_new, 0, true );
    mulSpectrums(F, F, B_new, 0, true );

    // update A ,B
    A = A*(1-rate) + A_new*rate;
    B = B*(1-rate) + B_new*rate;

    //Update C if contextAware
    if(contextAware){
        cv::Mat C_i,window,WINDOW_WARP;
        for(int i=0;i<4;i++){
            auto center2=roiCenter;
            if( i==0) center2.x+=patchSize.width;
            else if(i==1)center2.x-=patchSize.width;
            else if( i==2) center2.y+=patchSize.height;
            else if(i==3)center2.y-=patchSize.height;

            getRectSubPix(roi, patchSize, center2, window);
            if(window.channels()!=1)cvtColor(window, window, cv::COLOR_BGR2GRAY);
            preProcess(window, hanWin);
            dft(window, WINDOW_WARP, cv::DFT_COMPLEX_OUTPUT);
            mulSpectrums(WINDOW_WARP, WINDOW_WARP, C_i, 0, true);
            C+=C_i;
        }

        C = C*(1-rate) + C_new*rate;
        H = divDFTs(A, lambda1+B+lambda2*C);
    }
    else
        H = divDFTs(A, lambda1+B);

    Timer.add("track-marker");


    // Tracked rect
    double halfwindowSize = markerWarpSize*pow(pyrfactor, imgPyrIdx)*0.5;
    boundingBox = cv::Rect2d(cv::Point2d(center.x-halfwindowSize, center.y-halfwindowSize),
                             cv::Point2d(center.x+halfwindowSize, center.y+halfwindowSize));

    return true;
}

bool TrackerImpl::optimalScale(const cv::Mat& image, cv::Point2f& roiCenter)
{
    if(center.x < 0 || center.x>image.cols ||center.y<0 || center.y>image.rows)
        return  false;

    //Top and bottom level in the pyramid
    int tl = imgPyrIdx-int(multiScaleLevels/2);
    if(tl<0) tl = 0;
    int bl = imgPyrIdx+int(multiScaleLevels/2);

    cv::Mat region;
    cv::Size regionSize = cv::Size(int(floor(roiSize.width*pow(pyrfactor,imgPyrIdx))), int(floor(roiSize.height*pow(pyrfactor,imgPyrIdx))));
    getRectSubPix(image, regionSize, center, region);

    double bestPSR=0;
    cv::Point newDelta_xy(0,0);
    for(int p=tl; p<=bl; p++)
    {
        cv::Size nsize(regionSize.width/pow(pyrfactor,p), regionSize.height/pow(pyrfactor,p));
        cv::resize(region, region, nsize);

        cv::Mat image_sub;
        getRectSubPix(region, patchSize, cv::Point2d(region.rows*0.5,region.cols*0.5), image_sub);
//        cv::imshow("Pathc", image_sub);
        preProcess(image_sub, hanWin);
        cv::Point delta_xy;
        double PSR = correlate(image_sub, delta_xy, H);

//        std::cout <<PSR << std::endl;
//        cv::waitKey();

        if(PSR > bestPSR){
            bestPSR = PSR;
            imgPyrIdx = p;
            newDelta_xy = delta_xy;
            region.copyTo(roi);
        }
    }

    //Update ROI center
    roiCenter = cv::Point2d(roi.rows*0.5+newDelta_xy.x,roi.cols*0.5+newDelta_xy.y);

    //Delta in original image
    newDelta_xy = newDelta_xy*pow(pyrfactor,imgPyrIdx);

    //Apply shift
    center.x += newDelta_xy.x;
    center.y += newDelta_xy.y;
    for(auto &c:TheMarker){
        c.x += newDelta_xy.x;
        c.y += newDelta_xy.y;
    }

//    std::cout <<"PSR: "<< bestPSR << std::endl;
    if(center.x < 0 || center.x>image.cols ||center.y<0 || center.y>image.rows)
        return  false;
    if (bestPSR < psrThreshold)
        return false;

    return true;
}

//  Element-wise division of complex numbers in src1 and src2
cv::Mat TrackerImpl::divDFTs( const cv::Mat &src1, const cv::Mat &src2 ) const
{
    cv::Mat c1[2],c2[2],a1,a2,s1,s2,denom,re,im;

    // split into re and im per src
    cv::split(src1, c1);
    cv::split(src2, c2);

    // (Re2*Re2 + Im2*Im2) = denom
    //   denom is same for both channels
    cv::multiply(c2[0], c2[0], s1);
    cv::multiply(c2[1], c2[1], s2);
    cv::add(s1, s2, denom);

    // (Re1*Re2 + Im1*Im1)/(Re2*Re2 + Im2*Im2) = Re
    cv::multiply(c1[0], c2[0], a1);
    cv::multiply(c1[1], c2[1], a2);
    cv::divide(a1+a2, denom, re, 1.0 );

    // (Im1*Re2 - Re1*Im2)/(Re2*Re2 + Im2*Im2) = Im
    cv::multiply(c1[1], c2[0], a1);
    cv::multiply(c1[0], c2[1], a2);
    cv::divide(a1+a2, denom, im, -1.0);

    // Merge Re and Im back into a complex matrix
    cv::Mat dst, chn[] = {re,im};
    cv::merge(chn, 2, dst);
    return dst;
}

void TrackerImpl::preProcess( cv::Mat &window , cv::Mat &hw) const
{
    window.convertTo(window, CV_32F);
    log(window + 1., window);

    //normalize
    cv::Scalar mean,StdDev;
    meanStdDev(window, mean, StdDev);
    window = (window-mean[0]) / (StdDev[0]+eps);

    //Gaussain weighting
    window = window.mul(hw);
}

double TrackerImpl::correlate( const cv::Mat &image_sub, cv::Point &delta_xy, const cv::Mat& Hm) const
{
    cv::Mat IMAGE_SUB, RESPONSE, response;
    // filter in dft space
    dft(image_sub, IMAGE_SUB, cv::DFT_COMPLEX_OUTPUT);
    mulSpectrums(IMAGE_SUB, Hm, RESPONSE, 0, true );
    idft(RESPONSE, response, cv::DFT_SCALE|cv::DFT_REAL_OUTPUT);
    // update center position
    double maxVal; cv::Point maxLoc;
    minMaxLoc(response, 0, &maxVal, 0, &maxLoc);
    delta_xy.x = maxLoc.x - int(response.size().width/2);
    delta_xy.y = maxLoc.y - int(response.size().height/2);
    // normalize response
    cv::Scalar mean,std;
    meanStdDev(response, mean, std);
    return (maxVal-mean[0]) / (std[0]+eps); // PSR
}


cv::Mat TrackerImpl::randWarp( const cv::Mat& a ) const
{
    cv::RNG rng(8031965);

    // random rotation
    double C=0.1;
    double ang = rng.uniform(-C,C);
    double c=cos(ang), s=sin(ang);
    // affine warp matrix
    cv::Mat_<float> W(2,3);
    W << c + rng.uniform(-C,C), -s + rng.uniform(-C,C), 0,
            s + rng.uniform(-C,C),  c + rng.uniform(-C,C), 0;

    // random translation
    cv::Mat_<float> center_warp(2, 1);
    center_warp << a.cols/2, a.rows/2;
    W.col(2) = center_warp - (W.colRange(0, 2))*center_warp;

    cv::Mat warped;
    warpAffine(a, warped, W, a.size(), cv::BORDER_REFLECT);
    return warped;
}

void TrackerImpl::cornerUpsample(std::vector<cv::Point2f> &corners)
{
    if(corners.size()==0) return;

    for(size_t cIdx=0; cIdx<4; cIdx++)
    {
        cv::Size prevLowResSize= cornerPatchSize;
        std::vector<cv::Point2f> tmpcorner={cv::Point2f(cornerPatchSize.width/2, cornerPatchSize.height/2)};
        for(int curpyr=cornersPyrs[cIdx].size()-1;curpyr>=0;curpyr--)
        {
            float factor= float(cornersPyrs[cIdx][curpyr].cols)/float(prevLowResSize.width);
            int halfwsize=  0.5+2.5*factor;
            tmpcorner[0] = tmpcorner[0]*factor;
            cv::cornerSubPix( cornersPyrs[cIdx][curpyr],tmpcorner,cv::Size(halfwsize,halfwsize),cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::MAX_ITER , 4,0.5));
            prevLowResSize=cornersPyrs[cIdx][curpyr].size();

//            cv::Mat out;
//            cv::cvtColor(cornersPyrs[cIdx][curpyr], out, cv::COLOR_GRAY2BGR);
//            cv::circle(out,tmpcorner[0],3,cv::Scalar(0,0,255),-1);
//            cv::imshow("corner", out);
//            cv::waitKey();
        }

        //Update real corner in the original image
        corners[cIdx].x += tmpcorner[0].x - cornersPyrs[cIdx][0].cols/2;
        corners[cIdx].y += tmpcorner[0].y - cornersPyrs[cIdx][0].rows/2;

//        cv::Mat out;
//        cv::cvtColor(cornersPyrs[cIdx][0], out, cv::COLOR_GRAY2BGR);
//        cv::circle(out,tmpcorner[0],3,cv::Scalar(0,0,255),-1);
//        //cv::resize(out,out,cv::Size(1000,1000));
//        cv::imshow("corner", out);
//        cv::waitKey();
    }
}


void TrackerImpl::buildCornersPyramid(const cv::Mat& image)
{
    //Size of cornerPatchSize in the original image
    cv::Size nsize;
    nsize.width = int(ceil(cornerPatchSize.width*pow(pyrfactor,imgPyrIdx)));
    nsize.height = int(ceil(cornerPatchSize.height*pow(pyrfactor,imgPyrIdx)));

    //Numer of pyramid images
    int npyrimgs = 1;
    while (nsize.width/pow(cornersPyrFactor,npyrimgs) > cornerPatchSize.width)
        npyrimgs++;

    for(size_t cIdx=0; cIdx<4; cIdx++)
    {
        cornersPyrs[cIdx].resize(npyrimgs);
        getRectSubPix(image, nsize, TheMarker[cIdx], cornersPyrs[cIdx][0]);

        for(int i=1;i<npyrimgs;i++){
            double factor = pow(cornersPyrFactor,i);
            cv::Size ssize(cornersPyrs[cIdx][0].cols/factor, cornersPyrs[cIdx][0].rows/factor);
            cv::resize(cornersPyrs[cIdx][i-1],cornersPyrs[cIdx][i],ssize);
        }
    }
}
