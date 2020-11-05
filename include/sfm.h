#ifndef SFM_H
#define SFM_H

#include "common_include.h"

class SFM
{
public:
    typedef shared_ptr<SFM> Ptr;
    
    Mat img_1_;
    Mat img_2_;
    Mat K_;
    Mat R_;
    Mat t_;
    Eigen::Isometry3d   T_;
    
    vector<cv::KeyPoint>    keypoints_1_;
    vector<cv::KeyPoint>    keypoints_2_;
    vector<cv::Point3d>     pts_3d_;
    Mat                     descriptors_1_;
    Mat                     descriptors_2_;
    vector<cv::DMatch>      matches_;
    
    SFM();
    SFM(Mat K);
    ~SFM();
    
    void sfm();
    
private:
    void findFeatureMatch();
    cv::Point2d pixel2cam(const cv::Point2d& p);
    void poseEstimation2d2d();
    void triangulation();
    void bundleAdjustment();
};

#endif
