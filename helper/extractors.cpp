#include "extractors.h"

/**
 * \brief
 * Get Matched KeyPoints point data across two images
 * \param matchesImg Input vector<DMatch> KeyPoint Matches across img1 & img2
 * \param kpImg1 Input vector<KeyPoint> KeyPoints of image 1
 * \param kpImg2 Input vector<KeyPoint> KeyPoints of image 2
 * \param src Output vector<Point2d> the match.queryIdx.pt data from image 1
 * \param dst Output vector<Point2d> the match.trainIdx.pt data from image 2
 */
void sfm::xtr::srcDstPt(vector<DMatch> &matchesImg, vector<KeyPoint> &kpImg1, vector<KeyPoint> &kpImg2,
                        vector<Point2d> &src, vector<Point2d> &dst) {
    clog << msgs::xtr::LOG_EXTRACT_SRCDST << endl;
    for (const auto &m: matchesImg) {
        src.push_back(kpImg1[m.queryIdx].pt);
        dst.push_back(kpImg2[m.trainIdx].pt);
    }
}

/**
 * \brief
 * Filter inliers from all points with a mask
 * \param pt Input vector<Point2d> unfiltered 2D points
 * \param mask Input Mat to filter out irrelevant points
 * \param relevantPts Output vector<Point2d> filtered points
 */
void sfm::xtr::ptsRelevant(vector<Point2d> &pt, Mat &mask, vector<Point2d> &relevantPts) {
    // TODO: Maybe change Mat dims=[x, 1] mask to vector mask and run for auto
    clog << msgs::xtr::LOG_EXTRACT_PTS_RELEVANT << endl;
    for (int i = 0; i < mask.rows; ++i) {
        if (mask.at<int>(i, 0) > 0) {
            relevantPts.push_back(pt[i]);
        }
    }
}

/**
 * \brief
 * Extract the focal length from a camera matrix
 * \param K Input Mat Camera Matrix
 * \param f Output double focal length
 */
void sfm::xtr::focalLength(Mat &K, double &f) {
    clog << msgs::xtr::LOG_EXTRACT_FOCAL_LENGTH << endl;
    f = 0.5 * (K.at<double>(0, 0) + K.at<double>(1, 1));
}

/**
 * \brief
 * Extract the principal point from a camera matrix
 * \param K Input Mat Camera Matrix
 * \param pp Output Point2d principal point
 */
void sfm::xtr::principlePoint(Mat &K, Point2d &pp) {
    clog << msgs::xtr::LOG_EXTRACT_PRINCIPLE_POINT << endl;
    pp.x = K.at<double>(0, 2);
    pp.y = K.at<double>(1, 2);
}

/**
 * \brief
 * Extract simplified view (no skew, etc) from a given camera matrix
 * \param K Input Mat Original camera matrix
 * \param KIdeal Output Mat simplified form of K
 */
void sfm::xtr::KIdeal(Mat &K, Mat &KIdeal) {
    clog << msgs::xtr::LOG_EXTRACT_K_IDEAL << endl;
    double f;
    Point2d pp;
    focalLength(K, f);
    principlePoint(K, pp);
    KIdeal = (Mat_<double>(3, 3) << f, 0, pp.x, 0, f, pp.y, 0, 0, 1);
}


/**
 * \brief
 * Extract the rotation (R), translation (t) and inliers mask from a set of matched points
 * \param src Input vector<Point2d> the match.queryIdx.pt data from image N
 * \param dst Input vector<Point2d> the match.trainIdx.pt data from image N_next
 * \param K Input Mat camera matrix
 * \param R Output Mat Rotation
 * \param t Output Mat translation
 * \param mask Output Mat inliers mask/filter
 */
void sfm::xtr::RtMask(vector<Point2d> &src, vector<Point2d> &dst, Mat &K, Mat &R, Mat &t, Mat &mask) {
    clog << msgs::xtr::LOG_EXTRACT_RTMASK << endl;
    Mat E;
    ccv::findEssentialMat(src, dst, K, E, mask);
    ccv::recoverPose(E, src, dst, K, R, t, mask);
}

/**
 * \brief
 * Extract matched object and image points data,
 * given object and image matches after a reconstruction
 * \param kps Input vector<KeyPoint> keypoints data
 * \param matches Input vector<DMatch> matched keypoints
 * \param obj Input vector<Point3d> reconstructed object points data
 * \param matchedKpIdx Input vector<int> matched objectPointIdx and imageKeyPointIdx
 * \param objpts Output vector<Point3d> from matched object and image points, the object data
 * \param imgpts Output vector<Point2d> from matched object and image points, the image data
 */
void sfm::xtr::objImgPts(vector<KeyPoint> &kps, vector<DMatch> &matches, vector<Point3d> &obj,
                         vector<int> &matchedKpIdx,
                         vector<Point3d> &objpts, vector<Point2d> &imgpts) {
    clog << msgs::xtr::LOG_EXTRACT_OBJIMGPTS << endl;
    for (auto &match: matches) {
        auto src = match.queryIdx;
        auto dst = match.trainIdx;
        auto mkpidx = matchedKpIdx[src];
        if (mkpidx >= 0) {
            imgpts.push_back(kps[dst].pt);
            objpts.push_back(obj[mkpidx]);
        }
    }
}

void sfm::xtr::RtoEulerAngles(Mat &R, Vec3f &ea) {
    double sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6; // If true, the matrix is singular

    double x, y, z;
    if (!singular) {
        x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2(-R.at<double>(2, 0), sy);
        z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
        x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    // rad to deg
    ea = cv::Vec3f(x * 180.0 / CV_PI, y * 180.0 / CV_PI, z * 180.0 / CV_PI);
}
