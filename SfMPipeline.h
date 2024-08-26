//
// Created by kaloyantsankov - 2024
//

#ifndef SFMPIPELINE_H
#define SFMPIPELINE_H

#include <opencv2/opencv.hpp>
#include <future>

#include "helper/DataOps.h"
#include "helper/cameraCalibration.h"
#include "helper/Features.h"
#include "helper/extractors.h"
#include "helper/object.h"
#include "helper/visualiser.h"
#include "helper/config.h"
#include "helper/ccv.h"

namespace sfm {
    using namespace std;
    using namespace cv;

    class SfMPipeline {
    public:
        static void run();
    };
}

#endif //SFMPIPELINE_H
