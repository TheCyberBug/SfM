//
// Created by kaloyantsankov - 2024
//

#ifndef HERZ_H
#define HERZ_H
#include "../config.h"
using namespace sfm::vars;


/**
 * \brief Provides pre-configured options for running the SfM.
 */
namespace templates {
    inline void herz() {
        kcalib::CALIBRATE = false;
        ccv = true;
        dops::folderPath = "data/Herz-Jesus-P8/images/";
        dops::fileExt = ".jpg";
        dops::IMAGE_START = 0;
        dops::IMAGE_END = 0;
        dops::IMAGE_ZOOM = 1.0;
        dops::RESIZE_RATIO = 1.0;
        feat::MAX_THREADS = 3;
        obj::E_THRESHOLD_WIDTH = 0.50;
        obj::E_THRESHOLD_HEIGHT = 1.0;
    }

    inline void fountain() {
        kcalib::CALIBRATE = false;
        ccv = true;
        dops::folderPath = "data/fountain/images/";
        dops::fileExt = ".png";
        dops::IMAGE_START = 0;
        dops::IMAGE_END = 8;
        dops::IMAGE_ZOOM = 1.0;
        dops::RESIZE_RATIO = 1.0;
        feat::MAX_THREADS = 3;
        obj::E_THRESHOLD_WIDTH = 1.0;
        obj::E_THRESHOLD_HEIGHT = 1.5;
    }

    inline void statue() {
        kcalib::CALIBRATE = true;
        ccv = true;
        dops::folderPath = "data/sznw/subset1/";
        dops::fileExt = ".jpg";
        dops::IMAGE_START = 5;
        dops::IMAGE_END = 15;
        dops::RESIZE_RATIO = 0.5;
        dops::IMAGE_ZOOM = 2.0;
        feat::MAX_THREADS = 3;
        obj::E_THRESHOLD_WIDTH = 2.0;
        obj::E_THRESHOLD_HEIGHT = 2.0;
    }

    inline void creeper() {
        kcalib::CALIBRATE = true;
        ccv = true;
        dops::folderPath = "data/McCreeper/";
        dops::fileExt = ".jpg";
        dops::IMAGE_START = 0;
        dops::IMAGE_END = 0;
        dops::RESIZE_RATIO = 0.5;
        dops::IMAGE_ZOOM = 2.0;
        feat::MAX_THREADS = 3;
        obj::E_THRESHOLD_WIDTH = 0.5;
        obj::E_THRESHOLD_HEIGHT = 1.0;
    }
}

#endif //HERZ_H
