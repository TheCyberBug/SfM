//
// Created by kaloyantsankov - 2024.
//

#include "RecoverPose.h"

#include "RecoverPose.h"

void RecoverPose::recoverPoseFromEssential(const Mat &E,
                                           const vector<Point2d> &source,
                                           const vector<Point2d> &destination,
                                           const Mat &K,
                                           Mat &R, Mat &t, Mat &mask) {
    // SVD of essential
    Mat w, u, vt;
    SVD::compute(E, w, u, vt);

    // Check if first and second singular values are close (as they should be)
    if (fabsf(w.at<double>(0) - w.at<double>(1)) > 1e-07) {
        cerr << "Warning: First two singular values are not close." << endl;
    }

    Mat W = (Mat_<double>(3, 3) <<
             0, -1, 0,
             1, 0, 0,
             0, 0, 1);

    // all rotations and translations
    vector<Mat> Rs(4), ts(4);
    Rs[0] = u * W * vt;
    Rs[1] = u * W * vt;
    Rs[2] = u * W.t() * vt;
    Rs[3] = u * W.t() * vt;

    ts[0] = u.col(2);
    ts[1] = -u.col(2);
    ts[2] = u.col(2);
    ts[3] = -u.col(2);

    // proper rotation matrix? (det = 1)
    for (int i = 0; i < 4; ++i) {
        if (determinant(Rs[i]) < 0) {
            Rs[i] = -Rs[i];
            ts[i] = -ts[i];
        }
    }

    // pixel to normalized camera coordinates
    vector<Point2d> norm_source, norm_destination;
    undistortPoints(source, norm_source, K, Mat());
    undistortPoints(destination, norm_destination, K, Mat());

    // find best solution
    int max_inliers = 0;
    int best_solution = -1;
    vector<Mat> masks;
    for (int i = 0; i < 4; ++i) {
        masks.emplace_back(Mat(Size(1, source.size()), CV_8U));
    }

    Mat P1 = Mat::eye(3, 4, CV_64F); // Projection matrix for the first camera
    Mat P2(3, 4, CV_64F); // Projection matrix for the second camera

    // of all options
    for (int i = 0; i < 4; ++i) {
        hconcat(Rs[i], ts[i], P2); // Combine R and t into the projection matrix

        Mat points4D;
        triangulatePoints(P1, P2, norm_source, norm_destination, points4D);

        // chirality check
        int positive_depth_count = 0;
        for (int j = 0; j < points4D.cols; ++j) {
            Mat x = points4D.col(j);
            x.convertTo(x, CV_64F);
            x /= x.at<double>(3); // Convert to non-homogeneous coordinates

            // Check if point is in front of both cameras
            Mat x1 = P1 * x;
            Mat x2 = P2 * x;

            if (x1.at<double>(2) > 0 && x2.at<double>(2) > 0) {
                positive_depth_count++;
                masks[i].at<uchar>(j) = 1;
            }
        }

        // solution = max_inliers solution
        if (positive_depth_count > max_inliers) {
            max_inliers = positive_depth_count;
            best_solution = i;
        }
    }

    if (best_solution == -1) {
        cerr << "Error: No valid solution found." << endl;
        return;
    }

    // Set the output R and t
    R = Rs[best_solution];
    t = ts[best_solution];
    mask = masks[best_solution];
}
