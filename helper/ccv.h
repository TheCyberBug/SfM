#ifndef CCV_H
#define CCV_H
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <Eigen/Dense>
#include "opencv2/opencv.hpp"
#include "config.h"
#include "extractors.h"
#include "ccv/EssentialMatrix.h"
#include "ccv/RecoverPose.h"

using namespace std;
using namespace Eigen;
using namespace cv;

/**
 * \brief Custom implementations of some cv2 functions
 */
namespace sfm::ccv {
    void findEssentialMat(vector<Point2d> &src, vector<Point2d> &dst, Mat &K, Mat &E, Mat &mask);

    void recoverPose(const Mat &E, const vector<Point2d> &src, const vector<Point2d> &dst, const Mat &K, Mat &R, Mat &t,
                     Mat &mask);
}
#endif //CCV_H
