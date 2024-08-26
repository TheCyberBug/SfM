#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H

#include <opencv2/opencv.hpp>
#include "config.h"

using namespace cv;
using namespace std;

/**
 * \brief Camera Calibration
 */
namespace sfm::kcalib {
    /**
     * \brief Camera Calibration class for chessboard pattern calibration
     */
    class kCalib {
    public:
        void calibrate(Mat &K, Mat &distCoeff);
    };
}


#endif //CAMERACALIBRATION_H
