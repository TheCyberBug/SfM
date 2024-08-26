#ifndef OBJECT_H
#define OBJECT_H
#include <opencv2/opencv.hpp>
#include <math.h>
#include "extractors.h"
#include "config.h"

using namespace cv;
using namespace std;

namespace sfm::obj {
    /**
     * \brief
     * Encapsulates 3D object SfM related actions
     */
    class object {
    public:
        void update(Mat &K, vector<vector<KeyPoint> > &kpAll, vector<vector<DMatch> > &matchAll);

        void update(Mat &K, vector<KeyPoint> &kp1, vector<KeyPoint> &kp2, vector<DMatch> &match, int &matchIdx);

        void reconstruct(vector<Point2d> &src, vector<Point2d> &dst, Mat &K, Mat &Rinit, Mat &R,
                         Mat &tinit, Mat &t, vector<Point3d> &obj);

        void merge(vector<DMatch> &matches, vector<int> &matchedKpIdx, vector<int> &matchedKpIdxNew,
                   vector<Point3d> &obj, vector<Point3d> &obj_new);

        void bundleAdjustment(Mat &K, vector<Mat> &R, vector<Mat> &t, vector<vector<KeyPoint> > &kpImagesAll,
                              vector<Point3d> &obj, vector<vector<
                                  int> >
                              &matchedKpIdx);

        void ptAdjust(Mat &K, Mat &R, Mat &t, Point2f &imgPt, Point3d &objpt, Point3d &objpt_new);

        void removeStrayPoints(vector<Point3d> &obj, double threshold_multiplier = 1.5);

        void getColor(vector<Mat> &Images, vector<vector<KeyPoint> > kpAll, int objPoints, vector<Vec3b> &colorsAll);

        void deltaRt(Mat &R_average, Mat &t_average);

        [[nodiscard]] const std::vector<Point3d> &getObjData() const {
            return this->objData;
        }

        [[nodiscard]] const std::vector<vector<int> > &getMatchedKpIdx() const {
            return this->matchedKpIdx;
        }

        [[nodiscard]] const vector<Mat> &getR() const {
            return this->R;
        }

        [[nodiscard]] const vector<Mat> &getT() const {
            return this->t;
        }

    private:
        vector<Mat> R, t;
        vector<Point3d> objData;
        vector<vector<int> > matchedKpIdx;
    };
}

#endif //OBJECT_H
