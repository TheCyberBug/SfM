# 3D Reconstruction from Images

This project provides a software implementation for 3D reconstruction from images, based on the Structure-from-Motion (SfM) technique. The application works well with both wall-facing and turntable captured structure images.

## Features

- 3D point cloud reconstruction from a series of images
- Support for wall-facing and turntable captured images
- Camera calibration and image undistortion
- Bundle adjustment and point cloud cleaning
- Point cloud visualization and export

## Requirements

- C++17
- OpenCV
- VTK

## Algorithm Overview

The project follows a sequential algorithm:

1. Load Camera Matrix from file OR perform Camera Calibration
2. Undistort input images (if calibrated)
3. Extract KeyPoints and Descriptors for all images
4. Match features across images
5. Initial reconstruction:
   a. Compute Essential Matrix
   b. Extract R|t from Essential Matrix
   c. Filter relevant keypoints from images {0, 1}
   d. Create projection matrices using R|t for images 0 and 1
   e. Triangulate relevant keypoints using the projections
6. Continue reconstruction for remaining images:
   a. Generate 3D-2D point correspondences (current object and new image)
   b. Estimate R|t given the 3D-2D point correspondences
   c. Extract matched KeyPoints from the image pair
   d. Triangulate points
   e. Merge reconstructed objects
7. Perform bundle adjustment and point cleaning
8. Display the Point Cloud (VTK Engine)
9. Save the Point Cloud (.PLY format)

## Installation

Download and compile with CMake

## Usage

To run the program, open a terminal in the build directory and execute:

```
./SfM
```

### Custom Data

To use a custom dataset:

1. Add your dataset to the `data` folder.
2. When prompted by the program, specify the path to your custom dataset.

Required folder structure for custom datasets:

```
customDataFolder/
├── calib (optional)/
│   └── calibrationImages/
│       └── [calibration image files]
├── [input image files]
│
└── K.txt
```

## Performance

The application demonstrates reasonable reconstruction performance, utilizing OpenCV libraries for vision-related tasks and runtime acceleration. Multithreading is enabled wherever possible to improve performance.

## Limitations

Occasionally, the Essential Matrix is incorect which results into faulty reconstruction.

## Acknowledgements

This project was inspired by the following articles:
1. [Structure from Motion.](https://cmsc426.github.io/sfm/)
2. [How to use OpenCV. (n.d.). Reconstruction in OpenCV.](https://www.opencvhelp.org/tutorials/advanced/reconstruction-opencv/)

## Contact
Kaloyan Tsankov
ktsankov1@gmail.com
