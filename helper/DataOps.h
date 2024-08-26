#ifndef DATALOADER_H
#define DATALOADER_H

#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <vector>
#include <algorithm>

using namespace cv;
using namespace std;

namespace sfm::dops {
    class DataOps {
    public:
        void load(vector<Mat> &images, Mat &K);

        vector<string> filesList(const string &folderPath, const string &fileExt);

        void imagesLoad(vector<string> &imgList, vector<Mat> &imgContainer);

        void KLoad(string folderPath, Mat &K_CONTAINER);

        void exportToPly(const vector<Point3d> &points, const vector<Vec3b> &colors);
    };
}


#endif //DATALOADER_H
