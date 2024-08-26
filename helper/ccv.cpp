#include "ccv.h"

/**
 * \brief
 * Switcher function \n
 * if config.h -> ccv = true, perform the manual implementation, otherwise use cv::findEssentialMat \n\n
 * * Computes the Essential Matrix E and Inlier mask.
 * \param src Input vector of match.queryIdx.pt items
 * \param dst Input vector of match.trainIdx.pt items
 * \param K Input Mat Camera Intrinsic matrix
 * \param E Output Mat Essential matrix
 * \param mask Output Mat inlier points mask
 */
void sfm::ccv::findEssentialMat(vector<Point2d> &src,
                                vector<Point2d> &dst,
                                Mat &K, Mat &E, Mat &mask) {
    clog << msgs::ccv::LOG_CCV_E << endl;
    if (vars::ccv) {
        EssentialMatrix e;
        e.compute(src, dst, K, E, mask);
        //cout << "Essential Mat from Scratch" << E << endl;
        // Next time just do all in cv_64f from the beginning
        E.convertTo(E, CV_64F);
    } else {
        double f;
        Point2d pp;
        xtr::focalLength(K, f);
        xtr::principlePoint(K, pp);
        E = cv::findEssentialMat(src, dst, K, RANSAC, 0.999, 1.0, mask);
    }
}

/**
 * \brief
 * Switcher function\n
 * if config.h -> ccv = true, perform the manual implementation, otherwise use cv::recoverPose \n\n
 * Recovers rotation and translation from the Essential matrix\n
 * This function decomposes an essential matrix and then verifies possible poses by doing cheirality check.\n
 * The cheirality check means that the triangulated 3D points should have positive depth.\n
 * \param E Input Mat Essential matrix
 * \param src Input vector of match.queryIdx.pt items
 * \param dst Input vector of match.trainIdx.pt items
 * \param K Input Mat Camera Intrinsic Parameters
 * \param R Output Mat Rotation
 * \param t Output Mat Translation
 * \param mask Output Mat inlier points mask
 */
void sfm::ccv::recoverPose(const Mat &E,
                           const vector<Point2d> &src,
                           const vector<Point2d> &dst,
                           const Mat &K, Mat &R,
                           Mat &t, Mat &mask) {
    clog << msgs::ccv::LOG_CCV_POSE << endl;
    if (vars::ccv) {
        Mat rf, tf, nomask;
        RecoverPose rp;
        rp.recoverPoseFromEssential(E, src, dst, K, R, t, mask);
    } else {
        cv::recoverPose(E, src, dst, K, R, t, mask);
    }
}
