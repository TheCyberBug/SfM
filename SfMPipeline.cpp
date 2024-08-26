//
// Created by kaloyantsankov - 2024
//

#include "SfMPipeline.h"

/**
 * \brief The whole Structure-from-Motion 3D Reconstruction Pipeline:
 * \details:
 * -Loads the data
 * -Calibrates if required
 * -Feature extraction (multithreaded)
 * -Feature matching
 * -3D reconstruction
 * -Bundle adjustment
 * -Points filtering
 * -Coloured Visualisation
 * -Saves Object to file
 */
void sfm::SfMPipeline::run() {
    system("clear");
    // TODO: Load imgs dataloader; Move as separate foo; Call kCalib; Add bool to config AND auto switch when K not found!

    vector<Mat> images;
    auto K = Mat(3, 3, CV_64F);
    Mat distCoeff;
    dops::DataOps dataOps;
    dataOps.load(images, K);

    kcalib::kCalib kc;
    kc.calibrate(K, distCoeff);

    vector<Mat> undist;
    if (!distCoeff.empty())
        for (int i = 0; i < images.size(); ++i) {
            undist.emplace_back(Size(images[i].rows, images[i].cols), CV_64F);
            cv::undistort(images[i], undist[i], K, distCoeff);
        }
    if (!undist.empty())
        images = undist;

    //Feature extraction
    vector<vector<KeyPoint> > kpAll;
    vector<Mat> descAll;
    vector<vector<DMatch> > matchAll;
    feat::Features f;
    f.retrieveAndMatch(images, kpAll, descAll, matchAll);

    std::future<void> kpVis = std::async(std::launch::async, &feat::Features::viz, &f, images, kpAll, matchAll);

    obj::object obj;
    obj.update(K, kpAll, matchAll);
    for (int i = 1; i < matchAll.size(); ++i) {
        obj.update(K, kpAll[i], kpAll[i + 1], matchAll[i], i);
    }

    vector<Point3d> objData = obj.getObjData();
    vector<vector<int> > mKpIdx = obj.getMatchedKpIdx();
    vector<Mat> R = obj.getR();
    vector<Mat> t = obj.getT();

    obj.bundleAdjustment(K, R, t, kpAll, objData, mKpIdx);

    vector<Vec3b> colorsObj;
    obj.getColor(images, kpAll, objData.size(), colorsObj);

    // remove nan points in objData
    int i = 0;
    while (i < objData.size()) {
        if (std::isnan(objData[i].x)){// || colorsObj[i] == cv::Vec3b(-1, -1, -1)) {
            objData.erase(objData.begin() + i);
            colorsObj.erase(colorsObj.begin() + i);
            --i;
        }
        ++i;
    }

    vis::visualiser v;
    std::vector<Vec3b> nocolor;
    std::vector<Point3d> camera_positions;
    std::vector<Vec3f> camera_orientations;

    for (size_t idx = 0; idx < R.size(); ++idx) {
        cv::Mat RSingle = R[idx];
        Rodrigues(RSingle, RSingle);
        cv::Mat TSingle = t[idx];

        // cam t
        cv::Mat camera_position = -RSingle.t() * TSingle;
        camera_positions.push_back(cv::Point3d(camera_position.at<double>(0),
                                               camera_position.at<double>(1),
                                               camera_position.at<double>(2)));

        // cam R
        Vec3f co(RSingle.at<double>(2, 0), RSingle.at<double>(2, 1), RSingle.at<double>(2, 2));
        camera_orientations.push_back(co);
    }

    v.start(objData, colorsObj, camera_positions, camera_orientations);

    dataOps.exportToPly(objData, colorsObj);
}
