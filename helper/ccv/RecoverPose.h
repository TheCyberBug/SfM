//
// Created by kaloyantsankov - 2024.
//

#ifndef RECOVERPOSE_H
#define RECOVERPOSE_H
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

class RecoverPose {
public:
    void recoverPoseFromEssential(const Mat &E,
                                  const vector<Point2d> &source,
                                  const vector<Point2d> &destination, const Mat &K,
                                  Mat &R, Mat &t, Mat &mask);
};


#endif //RECOVERPOSE_H
