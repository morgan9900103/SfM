#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SE3;
using Sophus::SO3;

#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
using cv::Mat;

#include <vector>
#include <iostream>
#include <string>
#include <memory>

using namespace std;

#endif
