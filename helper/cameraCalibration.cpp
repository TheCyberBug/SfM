#include "cameraCalibration.h"

/**
 * \brief Camera Calibration via chessboard images\n
 * It loads the calibration images from the "calib" folder inside the specified data path\n
 * Example structure: dataFolder -> folder(calib), list(images)
 * \param K InputMat 3x3 Camera Intrinsic Parameters
 * \param distCoeff OutputMat distortion coefficients
 */
void sfm::kcalib::kCalib::calibrate(Mat &K, Mat &distCoeff) {
    if (vars::kcalib::CALIBRATE == false)
        return;
    clog << msgs::kcalib::LOG_CALIB_INIT << endl;
    Size cb(vars::kcalib::CHESSBOARD_WIDTH - 1, vars::kcalib::CHESSBOARD_HEIGHT - 1);

    vector<vector<Point3f> > objpts;
    vector<vector<Point2f> > imgpts;
    vector<Point3f> objp;

    for (int i = 0; i < cb.height; i++) {
        for (int j = 0; j < cb.width; j++) {
            objp.emplace_back(j, i, 0);
        }
    }

    vector<String> images;
    glob(vars::dops::folderPath + "calib/*" + vars::dops::fileExt, images);

    Mat img, gray;
    vector<Point2f> corners;

    for (int i = 0; i < images.size(); i++) {
        img = imread(images[i]);
        resize(img, img, Size(), vars::dops::RESIZE_RATIO, vars::dops::RESIZE_RATIO);
        cvtColor(img, gray, COLOR_BGR2GRAY);

        bool found = findChessboardCorners(gray, cb, corners,
                                           CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001));

            // Save the points
            imgpts.push_back(corners);
            objpts.push_back(objp);

            // Draw and display the corners
            drawChessboardCorners(img, cb, corners, found);
            Mat resized;
            resize(img, resized, Size(), 0.4, 0.4);
            imshow("Chessboard Corners", resized);
            waitKey(500); // Wait for 500ms
        }
    }
    destroyAllWindows();

    //Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;
    Size imageSize = img.size();

    calibrateCamera(objpts, imgpts, imageSize, K, distCoeff, rvecs, tvecs);
    K.at<double>(0, 0) *= vars::dops::IMAGE_ZOOM;
    K.at<double>(1, 1) *= vars::dops::IMAGE_ZOOM;

    clog << msgs::kcalib::LOG_CALIB_K << endl << K << endl;
    clog << msgs::kcalib::LOG_CALIB_DIST << endl << distCoeff << endl;
}
