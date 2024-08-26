//
// Created by kaloyantsankov - 2024.
//

#ifndef ESSENTIALMATRIX_H
#define ESSENTIALMATRIX_H
#include <opencv2/opencv.hpp>
#include <vector>
#include <random>

using namespace cv;
using namespace std;

class EssentialMatrix {
public:
    void compute(const std::vector<Point2d> &source,
                 const std::vector<Point2d> &destination,
                 const Mat &K,
                 Mat &E, Mat &mask);

    void getMask(const std::vector<Point2d> &source,
                 const std::vector<Point2d> &destination,
                 Mat &mask);

private:
    double symError(const Mat& E, const Point2d& x1, const Point2d& x2, const Mat& K_inv);
};

#endif //ESSENTIALMATRIX_H
