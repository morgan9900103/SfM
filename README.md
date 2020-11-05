# Structure from Motion

This is a project for SfM. Utilized series of 2D images to reconstruct a 3D structre of scene.

## Overview

the general steps of this method including:

1. Extract keypoints and keypoints matching
2. Pose estimation from two views
3. Triangulation
4. Minimized the reprojection error by nonlinear optimization

## Installation Dependencies

1. OpenCV
2. Eigen
3. G2O

## Building project

```bash
mkdir build
cd build
cmake ..
make
```

## To be continued

1. generate a point cloud for 3d points
