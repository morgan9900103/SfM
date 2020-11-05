#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "sfm.h"

SFM::SFM() : img_1_(cv::Mat()), img_2_(cv::Mat()), K_(Mat()), R_(Mat()), t_(Mat())
{
}

SFM::SFM(Mat K) : img_1_(cv::Mat()), img_2_(cv::Mat()), K_(K), R_(Mat()), t_(Mat())
{
}

SFM::~SFM()
{
}

void SFM::sfm()
{
    findFeatureMatch();
    poseEstimation2d2d();
    triangulation();
    bundleAdjustment();
}

void SFM::findFeatureMatch()
{
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    
    detector->detect(img_1_, keypoints_1_);
    detector->detect(img_2_, keypoints_2_);
    
    descriptor->compute(img_1_, keypoints_1_, descriptors_1_);
    descriptor->compute(img_2_, keypoints_2_, descriptors_2_);
    
    vector<cv::DMatch> match;
    matcher->match(descriptors_1_, descriptors_2_, match);
    
    double min_dist = 10000, max_dist = 0;
    
    for(int i = 0; i < descriptors_1_.rows; i++) {
        double dist = match[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }
    
    for(int i = 0; i < descriptors_1_.rows; i++) {
        if(match[i].distance <= max(2*min_dist, 30.0))
            matches_.push_back(match[i]);
    }
}

cv::Point2d SFM::pixel2cam(const cv::Point2d& p)
{
    return cv::Point2d(
        (p.x - K_.at<double>(0, 2)) / K_.at<double>(0, 0),
        (p.y - K_.at<double>(1, 2)) / K_.at<double>(1, 1)
    );
}

void SFM::poseEstimation2d2d()
{
    vector<cv::Point2d> points1;
    vector<cv::Point2d> points2;
    
    for(int i = 0; i < (int)matches_.size(); i++) {
        points1.push_back(keypoints_1_[matches_[i].queryIdx].pt);
        points2.push_back(keypoints_2_[matches_[i].trainIdx].pt);
    }
    
    // Essential matrix
    Mat essential_matrix;
    
    essential_matrix = findEssentialMat(points1, points2, K_);
    
    cv::recoverPose(essential_matrix, points1, points2, K_, R_, t_);
}

void SFM::triangulation()
{
    Mat T1 = (cv::Mat_<double>(3, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0);
    Mat T2 = (cv::Mat_<double>(3, 4) <<
        R_.at<double>(0, 0), R_.at<double>(0, 1), R_.at<double>(0, 2), t_.at<double>(0, 0),
        R_.at<double>(1, 0), R_.at<double>(1, 1), R_.at<double>(1, 2), t_.at<double>(1, 0),
        R_.at<double>(2, 0), R_.at<double>(2, 1), R_.at<double>(2, 2), t_.at<double>(2, 0));
        
    vector<cv::Point2d> pts_1, pts_2;
    for(cv::DMatch m : matches_) {
        pts_1.push_back(pixel2cam(keypoints_1_[m.queryIdx].pt));
        pts_2.push_back(pixel2cam(keypoints_2_[m.trainIdx].pt));
    }
    
    Mat pts_4d; // 3x4 homogemeous matrix
    triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);
    
    for(int i = 0; i < pts_4d.cols; i++) {
        Mat x = pts_4d.col(i);
        x /= x.at<double>(3, 0);
        cv::Point3d p(
            x.at<double>(0, 0),
            x.at<double>(1, 0),
            x.at<double>(2, 0));
        pts_3d_.push_back(p);
    }
}

void SFM::bundleAdjustment()
{
    vector<cv::Point2d> pts_2d;
    for(cv::DMatch m : matches_) {
        pts_2d.push_back(keypoints_2_[m.trainIdx].pt);
    }
    
    // Initialization
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
    unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverCSparse<Block::PoseMatrixType>());
    unique_ptr<Block> solver_ptr(new Block(move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(move(solver_ptr));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    
    // Vertex 
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();    // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<
        R_.at<double>(0, 0), R_.at<double>(0, 1), R_.at<double>(0, 2),
        R_.at<double>(1, 0), R_.at<double>(1, 1), R_.at<double>(1, 2),
        R_.at<double>(2, 0), R_.at<double>(2, 1), R_.at<double>(2, 2);
    pose->setId(0);
    pose->setEstimate(
        g2o::SE3Quat(
            R_mat,
            Vector3d(t_.at<double>(0, 0), t_.at<double>(1, 0), t_.at<double>(2, 0))));
    optimizer.addVertex(pose);
    
    int index = 1;
    
    // landmarks
    for(const cv::Point3d p : pts_3d_) {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId(index++);
        point->setEstimate(Vector3d(p.x, p.y, p.z));
        point->setMarginalized(true);
        optimizer.addVertex(point);
    }
    
    g2o::CameraParameters* camera = new g2o::CameraParameters(
        K_.at<double>(0, 0), Vector2d(K_.at<double>(0, 2), K_.at<double>(1, 2)), 0);
    
    camera->setId(0);
    optimizer.addParameter(camera);
    
    // edges 
    index = 1;
    for(const cv::Point2d p : pts_2d) {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId(index);
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(index)));
        edge->setVertex(1, pose);
        edge->setMeasurement(Vector2d(p.x, p.y));
        edge->setParameterId(0, 0);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }
    
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    
    T_ = Eigen::Isometry3d(pose->estimate());
    
    cout << endl << "after optimization:" << endl;
    cout << "T=" << T_.matrix() << endl;
}
