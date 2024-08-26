#include "object.h"


/**
 * \brief
 * Updates the 3D Reconstructed object with new data.\n
 * Call this when running update for the first time\n\n
 * Run getObjData() for object points
 * \param K Input Mat Camera Matrix
 * \param kpAll Input vector<vector<KeyPoint>> Key points of all images
 * \param matchAll Input vector<vector<DMatch>> list of matches across all images
 */
void sfm::obj::object::update(Mat &K, vector<vector<KeyPoint> > &kpAll, vector<vector<DMatch> > &matchAll) {
    clog << msgs::obj::LOG_OBJECT_UPDATE_FIRST << endl;
    // Extract src dst for match 0
    vector<Point2d> src, dst;
    xtr::srcDstPt(matchAll[0], kpAll[0], kpAll[1], src, dst);

    //Mat KIdeal;
    //xtr::KIdeal(K, KIdeal);

    //TODO: Refactor the ugly R|t mess
    R.push_back(Mat::eye(3, 3, CV_64F));
    t.push_back(Mat::zeros(3, 1, CV_64F));

    Mat R_init, t_init, mask;
    xtr::RtMask(src, dst, K, R_init, t_init, mask);
    R.push_back(R_init);
    t.push_back(t_init);

    vector<Point2d> srcr, dstr;
    xtr::ptsRelevant(src, mask, srcr);
    xtr::ptsRelevant(dst, mask, dstr);

    reconstruct(srcr, dstr, K, R[0], R[1], t[0], t[1], objData);

    //initial object
    // Init mkpidx
    for (auto &kps: kpAll) {
        matchedKpIdx.emplace_back(kps.size(), -1);
    }

    // valid matches => get valid kp idxs
    int m_counter = 0;
    auto &matches = matchAll[0];
    for (int i = 0; i < matches.size(); ++i) {
        if (mask.at<int>(i) == 0) {
            continue;
        }

        DMatch &match = matches[i];
        matchedKpIdx[0][match.queryIdx] = m_counter;
        matchedKpIdx[1][match.trainIdx] = m_counter;
        ++m_counter;
    }
}

/**
 * \brief
 * Updates the 3D reconstructed object with new pose and structure information:\n
 * Estimates the camera pose using PnP-RANSAC, updates the rotation and translation vectors,
 * reconstructs new 3D points, and merges them into the existing object data.\n\n
 * Run getObjData() for object points
 * \param K Input Mat Camera Matrix
 * \param kp1 Input vector<KeyPoint> key points of the Nth image
 * \param kp2 Input vector<KeyPoint> key points of the Nth+1 image
 * \param match Input vector<DMatch> matches between the Nth and Nth+1 images
 * \param matchIdx Input int idx of matches in the list of matches across all images
 */
//TODO: give matchIdx a new name
void sfm::obj::object::update(Mat &K,
                              vector<KeyPoint> &kp1, vector<KeyPoint> &kp2,
                              vector<DMatch> &match, int &matchIdx) {
    clog << msgs::obj::LOG_OBJECT_UPDATE << endl;
    vector<Point3d> objpts;
    vector<Point2d> imgpts;
    xtr::objImgPts(kp2, match, objData, matchedKpIdx[matchIdx], objpts, imgpts);

    Mat Rvec, rvec, tvec;
    cv::solvePnPRansac(objpts, imgpts, K, noArray(), rvec, tvec);
    cv::Rodrigues(rvec, Rvec);

    R.push_back(Rvec);
    t.push_back(tvec);

    vector<Point2d> srcloop, dstloop;
    xtr::srcDstPt(match, kp1, kp2, srcloop, dstloop);

    vector<Point3d> obj_new;
    reconstruct(srcloop, dstloop, K, R[matchIdx], Rvec, t[matchIdx], tvec, obj_new);
    merge(match, matchedKpIdx[matchIdx], matchedKpIdx[matchIdx + 1], objData, obj_new);
}

/**
 * \brief
 * Reconstructs 3D points from two sets of corresponding 2D points in different images.\n\n
 *
 * Takes in corresponding 2D points, the camera's intrinsic matrix, and the initial and
 * final rotation and translation matrices.\n
 * It then reconstructs the 3D points in space using triangulation.
 *
 * \param src Input vector<Point2d> the 2D points from the first image.
 * \param dst Input vector<Point2d> the 2D points from the second image.
 * \param K Input Mat camera intrinsic matrix.
 * \param Rinit Input Mat initial rotation matrix.
 * \param R Output Mat the rotation matrix after reconstruction.
 * \param tinit Input Mat the initial translation vector.
 * \param t Output Mat the translation vector after reconstruction.
 * \param obj Output vector<Point3d> the reconstructed 3D points.
 */
void sfm::obj::object::reconstruct(vector<Point2d> &src, vector<Point2d> &dst,
                                   Mat &K,
                                   Mat &Rinit, Mat &R,
                                   Mat &tinit, Mat &t,
                                   vector<Point3d> &obj) {
    clog << msgs::obj::LOG_OBJ_RECONSTRUCT << endl;
    // Init
    Mat proj_init = Mat::zeros(3, 4, CV_64F);
    Mat proj_next = Mat::zeros(3, 4, CV_64F);

    //// Copy R|t to projection mat
    Rinit.convertTo(proj_init(Rect(0, 0, 3, 3)), CV_64F);
    tinit.convertTo(proj_init(cv::Rect(3, 0, 1, 3)), CV_64F);
    //cout << Rinit << endl << tinit << endl << proj_init << endl;

    R.convertTo(proj_next(Rect(0, 0, 3, 3)), CV_64F);
    t.convertTo(proj_next(cv::Rect(3, 0, 1, 3)), CV_64F);

    //Apply intrinsic to projections
    K.convertTo(K, CV_64F);
    proj_init = K * proj_init;
    proj_next = K * proj_next;

    //Triangulate
    Mat s;
    //TODO: CCV This
    triangulatePoints(proj_init, proj_next, src, dst, s);

    //Convert structure points from homogenous 4D to 3D pts
    for (int i = 0; i < s.cols; ++i) {
        Mat col = s.col(i);
        col /= col.at<double>(3);
        Point3d p = {col.at<double>(0), col.at<double>(1), col.at<double>(2)};
        obj.push_back(p);
    }
}

/**
 * \brief
 * Merges newly reconstructed 3D points into the existing object model using keypoint matches.\n\n
 * Updates the correspondence between 2D keypoints and 3D object points.\n
 * If a keypoint has not been matched before, it adds the corresponding 3D point from the newly reconstructed set
 * to the existing 3D object points and updates the index mappings.
 *
 * \param matches Input vector<DMatch> image matches
 * \param matchedKpIdx Input vector<int> indices mapping keypoints in the first image to existing 3D points.
 * \param matchedKpIdxNew Output vector<int> indices that will map keypoints in the second image to 3D points.
 * \param obj Input/output vector of existing 3D object points.
 * \param obj_new Input vector of newly reconstructed 3D points to be merged.
 */
void sfm::obj::object::merge(vector<DMatch> &matches, vector<int> &matchedKpIdx, vector<int> &matchedKpIdxNew,
                             vector<Point3d> &obj, vector<Point3d> &obj_new) {
    for (int i = 0; i < matches.size(); ++i) {
        auto &src = matches[i].queryIdx;
        auto &dst = matches[i].trainIdx;
        auto &mkpidx = matchedKpIdx[src];
        if (mkpidx >= 0) {
            matchedKpIdxNew[dst] = mkpidx;
            continue;
        }
        obj.push_back(obj_new[i]);

        matchedKpIdx[src] = static_cast<int>(obj.size() - 1);
        matchedKpIdxNew[dst] = static_cast<int>(obj.size() - 1);
    }
}

/**
 * \brief
 * Bundle adjustment on the reconstructed 3D object points to refine the camera poses and 3D points.\n\n
 * Adjusts the 3D points and camera poses to minimize reprojection errors across all images.\n
 * Through the matched keypoints, adjusts the corresponding 3D points based on the current camera parameters.
 *
 * \param K Input Mat camera intrinsic matrix.
 * \param R Input/output vector<Mat> rotation matrices for each camera pose.
 * \param t Input/output vector<Mat> translation vectors for each camera pose.
 * \param kpImagesAll Input vector<vector<KeyPoint>> keypoints from all images.
 * \param obj Input/output vector<Point3d> 3D object points to be adjusted.
 * \param matchedKpIdx Input vector<vector<int>> indices mapping keypoints in each image to the corresponding 3D points.
 */
void sfm::obj::object::bundleAdjustment(Mat &K, vector<Mat> &R, vector<Mat> &t, vector<vector<KeyPoint> > &kpImagesAll,
                                        vector<Point3d> &obj, vector<vector<int> > &matchedKpIdx) {
    for (int i = 0; i < R.size(); ++i) {
        Mat r;
        Rodrigues(R[i], r);
        R[i] = r;
    }
    for (int i = 0; i < matchedKpIdx.size(); ++i) {
        auto &optsidx = matchedKpIdx[i];
        auto &kps = kpImagesAll[i];
        auto &R_ = R[i];
        auto &t_ = t[i];
        for (int j = 0; j < optsidx.size(); ++j) {
            auto &objptidx = optsidx[j];
            if (objptidx < 0) { continue; }
            Point3d ptadj;
            ptAdjust(K, R_, t_, kps[j].pt, obj[objptidx], ptadj);
            obj[objptidx] = ptadj;
        }
    }
}

/**
 * \brief
 * Adjusts a 3D object point based on its reprojection error with respect to a 2D image point.\n\n
 * Projects a 3D point into the 2D image plane using the current camera parameters.
 * It then calculates the reprojection error between the projected point and the actual 2D keypoint.
 * If the error exceeds predefined thresholds, the adjusted point is set to NaN to indicate an invalid adjustment.
 *
 * \param K Input Mat camera intrinsic matrix
 * \param R Input Mat rotation matrix (Rodrigues vector) for the current camera pose
 * \param t Input Mat translation vector for the current camera pose
 * \param imgPt Input Point2d image point corresponding to the 3D object point
 * \param objpt Input Point3d object point to be adjusted
 * \param objpt_new Output adjusted 3D object point. Set to NaN if the reprojection error exceeds the thresholds.
 */
void sfm::obj::object::ptAdjust(Mat &K, Mat &R, Mat &t, Point2f &imgPt, Point3d &objpt, Point3d &objpt_new) {
    vector<Point2d> projPts;
    vector<Point3d> obj = {objpt};
    cv::projectPoints(obj, R, t, K, noArray(), projPts);

    Point2f ppt = projPts[0];
    Point2f error = imgPt - ppt;

    if (abs(error.x) > vars::obj::E_THRESHOLD_WIDTH || abs(error.y) > vars::obj::E_THRESHOLD_HEIGHT) {
        objpt_new = Point3d(std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::quiet_NaN());
    } else { objpt_new = obj[0]; }
}

//obsolete, was used to filter out pts but gets carried away
void sfm::obj::object::removeStrayPoints(vector<Point3d> &obj, double threshold_multiplier) {
    if (obj.empty()) return;

    // Compute the centroid of the point cloud
    Point3d centroid(0, 0, 0);
    for (const auto &point: obj) {
        centroid += point;
    }
    centroid.x /= obj.size();
    centroid.y /= obj.size();
    centroid.z /= obj.size();

    // Calculate the distance of each point from the centroid
    vector<double> distances;
    for (const auto &point: obj) {
        double distance = sqrt(pow(point.x - centroid.x, 2) +
                               pow(point.y - centroid.y, 2) +
                               pow(point.z - centroid.z, 2));
        distances.push_back(distance);
    }

    // det a threshold distance
    double mean_dist = 0;
    for (double dist: distances) {
        mean_dist += dist;
    }
    mean_dist /= distances.size();

    double stddev_dist = 0;
    for (double dist: distances) {
        stddev_dist += pow(dist - mean_dist, 2);
    }
    stddev_dist = sqrt(stddev_dist / distances.size());

    double threshold = mean_dist + threshold_multiplier * stddev_dist;

    //filter out points > threshold
    vector<Point3d> filtered_obj;
    for (size_t i = 0; i < obj.size(); ++i) {
        if (distances[i] <= threshold) {
            filtered_obj.push_back(obj[i]);
        }
    }

    obj = filtered_obj;
}

/**
 * \brief
 * Extracts and assigns color to 3D object points based on their corresponding 2D keypoints in the images.\n\n
 *
 * \param Images Input vector<Mat> images used for reconstruction
 * \param kpAll Input vector<vector<KeyPoint>> key points from all images
 * \param objPoints Input int total 3D object points
 * \param colorsAll Output vector<Vec3b> the color for each 3D Point
 */
void sfm::obj::object::getColor(vector<Mat> &Images, vector<vector<KeyPoint> > kpAll, int objPoints,
                                vector<Vec3b> &colorsAll) {
    //colorsAll = vector<Vec3b>(objPoints, Vec3b(-1, -1, -1));
    colorsAll = vector<Vec3b>(objPoints, Vec3b(0, 0, 0));
    //TODO: nesting layers - too many
    for (int i = 0; i < matchedKpIdx.size(); ++i) {
        auto mkpidx = matchedKpIdx[i];

        Point2d pt;
        for (int j = 0; j < mkpidx.size(); ++j) {
            if (mkpidx[j] >= 0) {
                pt = kpAll[i][j].pt;
                pt = Point(cvRound(pt.x), cvRound(pt.y));
                // get whole part of double
                double x, y;
                modf(pt.x, &x);
                modf(pt.y, &y);
                if (pt.x >= 0 && pt.x < Images[i].cols && pt.y >= 0 && pt.y < Images[i].rows) {
                    colorsAll[mkpidx[j]] = Images[i].at<Vec3b>(pt);
                } else if (x >= 0 && x < Images[i].cols && y >= 0 && y < Images[i].rows) {
                    colorsAll[mkpidx[j]] = Images[i].at<Vec3b>(x, y);
                } else {
                    colorsAll[mkpidx[j]] = colorsAll[mkpidx[j - 1]];
                }
            }
        }
    }
}

void sfm::obj::object::deltaRt(Mat &R_average, Mat &t_average) {
    // Calculate average rotation
    int n = R.size() - 1;
    Mat R_sum = Mat::zeros(3, 3, CV_64F);
    for (int r = 1; r < R.size(); ++r) {
        R_sum += R[r];
    }
    R_average = R_sum / n;

    // Unsure whether I need it here
    // Ensure R_average is a valid rotation matrix
    // TODO: Test without it
    SVD svd(R_average);
    R_average = svd.u * svd.vt;

    // Calculate average translation
    Mat t_sum = Mat::zeros(3, 1, CV_64F);
    for (const Mat &tvec: t) {
        t_sum += tvec;
    }
    t_average = t_sum / n;
}
