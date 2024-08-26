#include "Features.h"


sfm::feat::Features::Features() {
    sift = SIFT::create(0, 3, 0.04, 10);
    bf = BFMatcher::create(NORM_L2);
}

/**
 * \brief
 * Extracts key points and descriptors for all images and provides their matches
 * \param images Input vector<Mat> images
 * \param K Input Mat Camera Matrix
 * \param distCoeff Input Mat Distortion Coefficients
 * \param kpAll Output vector<vector<KeyPoint>> KeyPoints for all images
 * \param descAll Output vector<Mat> Descriptors for all images
 * \param matchAll Output vector<vector<DMatch>> KeyPoints for all images
 */
void sfm::feat::Features::retrieveAndMatch(vector<Mat> &images,
                                           vector<vector<KeyPoint> > &kpAll, vector<Mat> &descAll,
                                           vector<vector<DMatch> > &matchAll) {
    kp_get(images, kpAll, descAll);
    match(descAll, matchAll);
}

/**
 * \brief
 * Extracts key points and descriptors of the target image with sift\n
 * Configured to run independently so it can be multithreaded
 * \param Image Input Mat target image
 * \param KpImage Output vector<KeyPoint> KeyPoints of the image
 * \param Descriptors Output Mat Descriptors of the image
 */
void sfm::feat::Features::kp_get_local(const Mat &Image, vector<KeyPoint> &KpImage, Mat &Descriptors) {
    clog << msgs::features::LOG_EXTRACT_SINGLE << endl;
    Ptr<Feature2D> sift_local = cv::SIFT::create(0, 3, 0.04, 10);

    Mat imgGray;
    cvtColor(Image, imgGray, cv::COLOR_BGR2GRAY);
    sift_local->detectAndCompute(imgGray, cv::noArray(), KpImage, Descriptors);
    --threads;
}

/**
 * \brief
 * Extracts key points and descriptors of the target image with sift\n
 * \param Image Input Mat target image
 * \param KpImage Output vector<KeyPoint> KeyPoints of the image
 * \param Descriptors Output Mat Descriptors of the image
 */
void sfm::feat::Features::kp_get(Mat &Image, vector<KeyPoint> &KpImage, Mat &Descriptors) {
    //Ptr<Feature2D> sift_local = SIFT::create(0, 3, 0.04, 10);
    clog << sfm::msgs::features::LOG_EXTRACT_SINGLE << endl;
    Mat imgGray = Image;
    cvtColor(Image, imgGray, COLOR_BGR2GRAY);
    sift->detectAndCompute(imgGray, noArray(), KpImage, Descriptors);
}

/**
 * \brief
 * Extract KeyPoints and Descriptors across a list of images.\n
 * The function is multithreaded.\n\n
 * N of threads can be set in config.h -> vars::feat::MAX_THREADS
 * \warning Each thread consumes significant RAM amount. To avoid crashes, reduce the threads allowed!
 * \param Images Input vector<Mat> list of images
 * \param KpImagesAll Output vector<vector<KeyPoint>> KeyPoints of all images
 * \param DescImagesAll Output vector<Mat> Descriptors of all images
 */
void sfm::feat::Features::kp_get(vector<Mat> &Images, vector<vector<KeyPoint> > &KpImagesAll,
                                 vector<Mat> &DescImagesAll) {
    auto start_time = std::chrono::high_resolution_clock::now();
    clog << sfm::msgs::features::LOG_EXTRACT_BUNDLE << endl;
    KpImagesAll = vector<vector<KeyPoint> >(Images.size());
    DescImagesAll = vector<Mat>(Images.size());

    // Multi-Threading -- halved the req time to compute
    std::vector<std::future<void> > futures;
    for (size_t i = 0; i < Images.size(); ++i) {
        while (threads >= vars::feat::MAX_THREADS); // { std::this_thread::yield(); }; - slows down
        // Hold until there's free thread available
        // Asynchronously run kp_get for each image
        ++threads;
        futures.push_back(std::async(std::launch::async, [&Images, &KpImagesAll, &DescImagesAll, i]() {
            feat::Features::kp_get_local(Images[i], KpImagesAll[i], DescImagesAll[i]);
        }));
    }

    // Wait for all futures to complete
    for (auto &fut: futures) {
        fut.get();
    }

    // TODO: May produce an error; Fix for async
    // if (!kpimg.empty() && !descimg.empty()) {
    //     KpImagesAll[i] = kpimg;
    //     DescImagesAll[i] = descimg;
    // } else { cerr << msgs::features::ERR_EXTRACTION_FAIL << endl; }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    clog << "Total execution time: " << duration.count() << " milliseconds" << endl;
}

/**
 * \brief
 * BFMatcher: matches Image 1 with Image 2
 * \param DescriptorImg1 Input Mat Descriptors of Image 1
 * \param DescriptorImg2 Input Mat Descriptors of Image 2
 * \param KNNMatches Output vector<DMatch> matches between Image 1 & 2
 */
void sfm::feat::Features::match(Mat &DescriptorImg1, Mat &DescriptorImg2, vector<DMatch> &KNNMatches) {
    clog << sfm::msgs::features::LOG_MATCH_SINGLE << endl;
    vector<vector<DMatch> > BFMatches;
    bf->knnMatch(DescriptorImg1, DescriptorImg2, BFMatches, 2);

    // Filter Matches Lowe's Law
    for (auto &match: BFMatches) {
        if (match[0].distance < 0.7 * match[1].distance) {
            KNNMatches.push_back(match[0]);
        }
    }

    if (KNNMatches.empty())
        cerr << sfm::msgs::features::ERR_MATCH_FAIL << endl;
}

/**
 * \brief
 * BFMatcher: matches images from their descriptors (ImageN and its successor)\n\n
 * For a set of images N, produces a KNNMatches vector of size N-1
 * \param DescriptorsImgs Input vector<Mat> list of Descriptors of all Images
 * \param KNNMatches Output vector<vector<DMatch>> matches between ImageN and the next in the line
 */
void sfm::feat::Features::match(vector<Mat> &DescriptorsImgs, vector<vector<DMatch> > &KNNMatches) {
    // REMEMBER KNNMATCHES.SIZE() = DESCRIPTORS.SIZE() - 1 = 7
    clog << sfm::msgs::features::LOG_MATCH_BUNDLE << endl;
    for (size_t i = 0; i < (DescriptorsImgs.size() - 1); ++i) {
        vector<DMatch> KNNMatchSingle;
        match(DescriptorsImgs[i], DescriptorsImgs[i + 1], KNNMatchSingle);

        if (!KNNMatchSingle.empty()) {
            KNNMatches.push_back(KNNMatchSingle);
        }
    }
}

/**
 * \brief
 * Visualises the matched key points between an image from the list and its successor
 * \param images Input vector<Mat> list of Images
 * \param kpAll Input vector<vector<KeyPoint>> list of KeyPoints for all images
 * \param matchAll Input vector<vector<DMatch>> list of matches across all images
 */
void sfm::feat::Features::viz(const vector<Mat> &images, const vector<vector<KeyPoint> > &kpAll,
                              const vector<vector<DMatch> > &matchAll) {
    if (vars::viz) {
        for (size_t i = 0; i < images.size() - 1; ++i) {
            // Take the first image and its matches with the next image
            const Mat &img1 = images[i];
            const Mat &img2 = images[i + 1];

            const vector<KeyPoint> &kp1 = kpAll[i];
            const vector<KeyPoint> &kp2 = kpAll[i + 1];

            const vector<DMatch> &matches = matchAll[i];

            // Draw matches
            Mat imgMatches;
            drawMatches(img1, kp1, img2, kp2, matches, imgMatches);
            // TODO: From config
            resize(imgMatches, imgMatches, Size(), 0.25, 0.25);

            // Display the matches
            string windowName = "Matches between Image " + to_string(i) + " and Image " + to_string(i + 1);
            namedWindow(windowName, WINDOW_NORMAL);
            imshow(windowName, imgMatches);
            // TODO: From config
            waitKey(0);
            destroyWindow(windowName);
        }
        destroyAllWindows();
    }
}
