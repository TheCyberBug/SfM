#ifndef FEATURES_H
#define FEATURES_H
#include <opencv2/opencv.hpp>
#include <future>
#include "config.h"

using namespace std;
using namespace cv;

namespace sfm::feat {
    /**
     * \brief Provides extraction, matching and visualisation of image features
     */
    class Features {
    public:
        Features();

        void retrieveAndMatch(vector<Mat> &images,
                              vector<vector<KeyPoint>> &kpAll, vector<Mat> &descAll, vector<vector<DMatch>> &matchAll);

        static void kp_get_local(const Mat &Image, vector<KeyPoint> &KpImage, Mat &Descriptors);

        void kp_get(Mat &Image, vector<KeyPoint> &KpImage, Mat &Descriptors);

        void kp_get(vector<Mat> &Images, vector<vector<KeyPoint> > &KpImagesAll, vector<Mat> &DescImagesAll);

        void match(Mat &DescriptorImg1, Mat &DescriptorImg2, vector<DMatch> &KNNMatches);

        void match(vector<Mat> &DescriptorsImgs, vector<vector<DMatch> > &KNNMatches);

        void viz(const vector<Mat> &images, const vector<vector<KeyPoint> > &kpAll,
                 const vector<vector<DMatch> > &matchAll);

    private:
        Ptr<Feature2D> sift;
        Ptr<ORB> orb = ORB::create();

        Ptr<BFMatcher> bf;
        FlannBasedMatcher flann;
    };
    inline int threads = 0;
}
#endif //FEATURES_H
