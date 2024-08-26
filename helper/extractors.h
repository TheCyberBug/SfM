#ifndef EXTRACTORS_H
#define EXTRACTORS_H
#include <opencv2/opencv.hpp>
#include "config.h"
#include "ccv.h"

using namespace std;
using namespace cv;

/**
 * \brief Provides helper data extractor and converter methods
 */
namespace sfm::xtr {
    void srcDstPt(vector<DMatch> &matchesImg, vector<KeyPoint> &kpImg1, vector<KeyPoint> &kpImg2, vector<Point2d> &src,
                  vector<Point2d> &dst);

    void ptsRelevant(vector<Point2d> &pt, Mat &mask, vector<Point2d> &relevantPts);

    void focalLength(Mat &K, double &f);

    void principlePoint(Mat &K, Point2d &pp);

    void KIdeal(Mat &K, Mat &KIdeal);

    void RtMask(vector<Point2d> &src, vector<Point2d> &dst, Mat &K, Mat &R, Mat &t, Mat &mask);

    void objImgPts(vector<KeyPoint> &kps, vector<DMatch> &matches, vector<Point3d> &obj, vector<int> &matchedKpIdx,
                   vector<Point3d> &objpts, vector<Point2d> &imgpts);

    void RtoEulerAngles(Mat &R, Vec3f &ea);

    void matchColor(vector<Mat> imgs, vector<vector<KeyPoint>> kpsAll, vector<vector<DMatch>> matchesAll, vector<vector<Scalar>> colorsAll);
}

#endif //EXTRACTORS_H
