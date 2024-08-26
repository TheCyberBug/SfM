#ifndef CONFIG_H
#define CONFIG_H
#include <string>
#include <opencv2/core/mat.hpp>

using namespace std;
/**
 * \brief Stores function messages
 */
namespace sfm::msgs {
    /**
     * \brief messages of the features class
     */
    namespace dops {
        inline string LOG_ACTION_SAVEPLY = "Saved .ply file";
    }
    namespace kcalib {
        inline string LOG_CALIB_INIT = "CAMERA_CALIBRATION: INITIALISE";
        inline string LOG_CALIB_K = "CAMERA_CALIBRATION_K_MATRIX: ";
        inline string LOG_CALIB_DIST = "CAMERA_CALIBRATION_DISTORTION_COEFFICIENTS: ";
    }
    namespace features {
        inline string LOG_EXTRACT_SINGLE = "FEATURES_ACTION: EXTRACT_SINGLE";
        inline string LOG_EXTRACT_BUNDLE = "FEATURES_ACTION: EXTRACT_BUNDLE";
        inline string LOG_MATCH_SINGLE = "MATCHING_ACTION: BF_SINGLE";
        inline string LOG_MATCH_BUNDLE = "MATCHING_ACTION: BF_BUNDLE";

        inline string ERR_EXTRACTION_FAIL = "FEATURES_ACTION_SKIP: Could NOT detect features for the image.";
        inline string ERR_MATCH_FAIL = "MATCHING_ACTION_SKIP: Could NOT match.";
    }
    namespace obj {
        inline string LOG_OBJECT_UPDATE_FIRST = "OBJECT_ACTION: UPDATE|INITIAL";
        inline string LOG_OBJECT_UPDATE = "OBJECT_ACTION: UPDATE|SUBSEQUENT";
    }
    /**
     * \brief messages of the extractor class
     */
    namespace xtr {
        inline string LOG_EXTRACT_SRCDST = "EXTRACTOR_ACTION: EXTRACT_SRC_DST";
        inline string LOG_EXTRACT_PRINCIPLE_POINT = "EXTRACTOR_ACTION: EXTRACT_PRINCIPLE_POINT";
        inline string LOG_EXTRACT_FOCAL_LENGTH = "EXTRACTOR_ACTION: EXTRACT_FOCAL_LENGTH";
        inline string LOG_EXTRACT_K_IDEAL = "EXTRACTOR_ACTION: COMPUTE_CAMERA_MATRIX_IDEAL";
        inline string LOG_EXTRACT_RTMASK = "EXTRACTOR_ACTION: FIND_R|t_AND_MASK";
        inline string LOG_EXTRACT_PTS_RELEVANT = "EXTRACTOR_ACTION: FILTER_SRC|DST_BY_MASK";
        inline string LOG_EXTRACT_OBJIMGPTS = "EXTRACTOR_ACTION: GET_OBJECT_IMAGE_PAIRED_POINTS";
    }
    /**
     * \brief messages of the custom cv implementations
     */
    namespace ccv {
        inline string LOG_CCV_E = "CUSTOM_CV_FUNC: FIND_ESSENTIAL_MAT";
        inline string LOG_CCV_POSE = "CUSTOM_CV_FUNC: RECOVER_POSE";
    }
    /**
     * \brief messages of the object class
     */
    namespace obj {
        inline string LOG_OBJ_RECONSTRUCT = "OBJECT_ACTION: RECONSTRUCT";
    }
}
/**
 * \brief global config parameters
 */
namespace sfm::vars {
    inline bool viz = false;
    inline bool ccv = false;
    /**
     * \brief config of dataOperations
     */
    namespace dops {
        inline string folderPath; // param sets on UI calls
        inline string fileExt;
        inline int IMAGE_START = 0;
        inline int IMAGE_END = 0; // if end = 0 then load to the images.size()
        inline double RESIZE_RATIO = 1.0; // affects performance, changes K

        inline string outputFilename = "data/out.ply";
        inline double IMAGE_ZOOM = 1.0;
    }
    /**
     * \brief config of cameraCalibration class
     */
    namespace kcalib {
        // whether to run calibration
        inline bool CALIBRATE = false;

        // where to look for calibration data
        inline string CHESSBOARD_IMAGES_PATH = "data/calib";
        inline string CHESSBOARD_IMAGES_EXTENSION = ".jpg";

        // N of inner corners in calib photos
        inline int CHESSBOARD_WIDTH = 10;
        inline int CHESSBOARD_HEIGHT = 7;
    }
    /**
     * \brief config of features class
     */
    namespace feat {
        // Hyper-threading
        inline int MAX_THREADS = 3;
    }
    /**
     * \brief config of object class
     */
    namespace obj {
        // threshold for filtering stray points
        inline double E_THRESHOLD_WIDTH = 0.50;
        inline double E_THRESHOLD_HEIGHT = 1.0;
    }
}

#endif //CONFIG_H
