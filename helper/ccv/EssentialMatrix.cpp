//
// Created by kaloyantsankov - 2024.
//

#include "EssentialMatrix.h"

void EssentialMatrix::compute(const vector<Point2d> &source, const vector<Point2d> &destination,
                              const Mat &K, Mat &E, Mat &mask) {
    const int numPoints = source.size();
    int k = 5;  // minimum number of points / degrees of freedom
    int numIterations = log(1.0 - 0.999) / log(1.0 - pow(0.5, k));
    //cout << numIterations << endl;
    const double inlierThreshold = 0.0010;
    //const double inlierThreshold = 1.0;

    Mat bestE;
    Mat bestMask;
    int maxInliers = 0;

    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(0, numPoints - 1);

    Mat K_inv = K.inv();

    // RANSAC
    int subset_size = 8;
    for (int iter = 0; iter < numIterations; ++iter) {
        // Randomly select 8 point correspondences
        vector<Point2d> srcSubset(subset_size), dstSubset(subset_size);
        for (int i = 0; i < subset_size; ++i) {
            int idx = dis(gen);
            srcSubset[i] = source[idx];
            dstSubset[i] = destination[idx];
        }

        // normalise
        vector<Point2d> srcNormalized(subset_size), dstNormalized(subset_size);
        for (int i = 0; i < subset_size; ++i) {
            Mat srcHom = (Mat_<double>(3, 1) << srcSubset[i].x, srcSubset[i].y, 1);
            Mat dstHom = (Mat_<double>(3, 1) << dstSubset[i].x, dstSubset[i].y, 1);

            Mat srcNorm = K_inv * srcHom;
            Mat dstNorm = K_inv * dstHom;

            srcNormalized[i] = Point2d(srcNorm.at<double>(0) / srcNorm.at<double>(2),
                                       srcNorm.at<double>(1) / srcNorm.at<double>(2));
            dstNormalized[i] = Point2d(dstNorm.at<double>(0) / dstNorm.at<double>(2),
                                       dstNorm.at<double>(1) / dstNorm.at<double>(2));
        }

        // Ax = 0
        Mat A(subset_size, 9, CV_64F);
        for (int i = 0; i < subset_size; ++i) {
            double x1 = srcNormalized[i].x, y1 = srcNormalized[i].y;
            double x2 = dstNormalized[i].x, y2 = dstNormalized[i].y;

            A.at<double>(i, 0) = x2 * x1;
            A.at<double>(i, 1) = x2 * y1;
            A.at<double>(i, 2) = x2;
            A.at<double>(i, 3) = y2 * x1;
            A.at<double>(i, 4) = y2 * y1;
            A.at<double>(i, 5) = y2;
            A.at<double>(i, 6) = x1;
            A.at<double>(i, 7) = y1;
            A.at<double>(i, 8) = 1;
        }

        // Solve Ax = 0 using SVD
        Mat w, u, vt;
        SVD::compute(A, w, u, vt, SVD::FULL_UV);

        // last column of V => solution
        Mat E_estimated = vt.row(8).reshape(0, 3);

        // internal constraint - two singular values are equal and one is zero
        SVD::compute(E_estimated, w, u, vt);
        w.at<double>(0) = (w.at<double>(0) + w.at<double>(1)) / 2;
        w.at<double>(1) = w.at<double>(0);
        w.at<double>(2) = 0;
        E_estimated = u * Mat::diag(w) * vt;

        // Count inliers
        int inliers = 0;
        Mat currMask(Size(1, static_cast<int>(source.size())), CV_8U, Scalar(0));
        for (int i = 0; i < numPoints; ++i) {
            double error = symError(E_estimated, source[i], destination[i], K_inv);
            // OLD no asym error
            // Mat x1 = (Mat_<double>(3, 1) << source[i].x, source[i].y, 1);
            // Mat x2 = (Mat_<double>(3, 1) << destination[i].x, destination[i].y, 1);
            //
            // x1 = K_inv * x1;
            // x2 = K_inv * x2;
            //
            // x1 /= x1.at<double>(2);
            // x2 /= x2.at<double>(2);
            //
            // Mat epipolarLine = E_estimated * x1;
            // double error = abs(x2.dot(epipolarLine)) /
            //                sqrt(epipolarLine.at<double>(0) * epipolarLine.at<double>(0) +
            //                     epipolarLine.at<double>(1) * epipolarLine.at<double>(1));

            if (error < inlierThreshold) {
                currMask.at<char>(0, i) = 1;
                inliers++;
            }
        }

        if (inliers > maxInliers) {
            maxInliers = inliers;
            bestE = E_estimated.clone();
            bestMask = currMask.clone();
        }
    }

    E = bestE.clone();
    mask = bestMask.clone();
}

double EssentialMatrix::symError(const Mat &E, const Point2d &x1, const Point2d &x2, const Mat &K_inv) {
    // points to homogeneous, normalize using K_inv
    Mat x1_h = (Mat_<double>(3, 1) << x1.x, x1.y, 1);
    Mat x2_h = (Mat_<double>(3, 1) << x2.x, x2.y, 1);

    Mat x1_norm = K_inv * x1_h;
    Mat x2_norm = K_inv * x2_h;

    x1_norm /= x1_norm.at<double>(2);
    x2_norm /= x2_norm.at<double>(2);

    // epi lines
    Mat l2 = E * x1_norm; // epi line in image 2 for point x1
    Mat l1 = E.t() * x2_norm; // epi line in image 1 for point x2


    double symmetricError = (x2_norm.dot(E * x1_norm) + x1_norm.dot(E.t() * x2_norm)) /
                        (l1.dot(l1) + l2.dot(l2));

    // TODO: REMOVE - Old compute
    // Calculate distances from points to corresponding epipolar lines
    double d1 = abs(x1_norm.dot(l1)) / sqrt(l1.at<double>(0) * l1.at<double>(0) + l1.at<double>(1) * l1.at<double>(1));
    double d2 = abs(x2_norm.dot(l2)) / sqrt(l2.at<double>(0) * l2.at<double>(0) + l2.at<double>(1) * l2.at<double>(1));

    // Return the symmetrical error (sum of distances)
    //return d1 + d2;
    return abs(symmetricError);
}
