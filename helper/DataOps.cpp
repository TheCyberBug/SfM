#include "DataOps.h"

#include "config.h"

/**
 * \brief
 * Load images and Camera intrinsics from files
 * \param images Output vector<Mat> - Stores all images
 * \param K Output Mat - Camera Intrinsic Matrix
 */
void sfm::dops::DataOps::load(vector<Mat> &images, Mat &K) {
    vector<string> files = filesList(vars::dops::folderPath, vars::dops::fileExt);
    for (auto &file: files) {
        cout << file << endl;
    }

    imagesLoad(files, images);


    KLoad(vars::dops::folderPath + "/K.txt", K);
}

/**
 * \brief
 * Provides a list of photos that comply with the setting - folderPath, fileExt
 * \param folderPath Input string - path to images files
 * \param fileExt Input string - photos' extension (.jpg, .png, etc)
 * \return Output vector<string> - list of files
 */
vector<string> sfm::dops::DataOps::filesList(const string &folderPath, const string &fileExt) {
    vector<string> filenames;

    for (const auto &entry: filesystem::directory_iterator(folderPath)) {
        if (entry.is_regular_file() && entry.path().extension() == fileExt) {
            filenames.push_back(entry.path().string());
        }
    }

    // Sort filenames (default lexicographical order which sorts by name)
    sort(filenames.begin(), filenames.end());

    return filenames;
}


/**
 * \brief
 * Loads the images given a set of files\n
 * The range [start: stop] can be set in config.h -> vars::dl::IMAGE_START & vars::dl::IMAGE_END
 * \param imgFiles Input vector<string> - list of files to load
 * \param imgContainer Output vector<Mat> - stores the loaded images into it
 */
void sfm::dops::DataOps::imagesLoad(vector<string> &imgFiles, vector<Mat> &imgContainer) {
    int i = 0;
    if (vars::dops::IMAGE_END == 0)
        vars::dops::IMAGE_END = imgFiles.size();
    for (const auto &imgFile: imgFiles) {
        if (i < vars::dops::IMAGE_START || i > vars::dops::IMAGE_END) {
            i++;
            continue;
        }
        Mat imgtemp = imread(imgFile);
        resize(imgtemp, imgtemp, Size(), sfm::vars::dops::RESIZE_RATIO, sfm::vars::dops::RESIZE_RATIO);
        imgContainer.push_back(imgtemp);
        i++;
    }
}

/**
 * \brief Load the Camera Matrix from file
 * \param K_FILEPATH Input string - path to Camera Intrinsic  file
 * \param K_CONTAINER Output Mat - Camera Intrinsics
 */
void sfm::dops::DataOps::KLoad(string K_FILEPATH, Mat &K_CONTAINER) {
    std::ifstream file(K_FILEPATH);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << K_FILEPATH << std::endl;
        exit(0);
    }
    try {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                file >> K_CONTAINER.at<double>(i, j);
            }
        }
        //TODO: fix for f
        K_CONTAINER = K_CONTAINER * sfm::vars::dops::RESIZE_RATIO;
        K_CONTAINER.at<double>(2, 2) = 1.0;
        cout << "K :" << K_CONTAINER << endl;
    } catch (Exception e) {
        cerr << "Couldn't load the camera matrix: " << endl << e.err << endl;
    }
}

void sfm::dops::DataOps::exportToPly(const vector<Point3d> &points,
                                     const vector<Vec3b> &colors) {
    clog << msgs::dops::LOG_ACTION_SAVEPLY << endl << endl;
    std::ofstream outFile(vars::dops::outputFilename);
    outFile << "ply\nformat ascii 1.0\n";
    outFile << "element vertex " << points.size() << "\n";
    outFile << "property float x\nproperty float y\nproperty float z\n";
    outFile << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    outFile << "end_header\n";

    for (size_t i = 0; i < points.size(); ++i) {
        outFile << points[i].x << " " << points[i].y << " " << points[i].z << " "
                << (int) colors[i][2] << " " << (int) colors[i][1] << " " << (int) colors[i][0] << "\n";
    }

    outFile.close();
}
