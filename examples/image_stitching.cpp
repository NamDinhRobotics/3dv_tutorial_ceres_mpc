#include "opencv2/opencv.hpp"

int main()
{
    // Load two images (c.f. We assume that two images have the same size and type)
    cv::Mat image1 = cv::imread("/home/dinhnambkhn/3dv_tutorial/bin/data/hill01.jpg");
    cv::Mat image2 = cv::imread("/home/dinhnambkhn/3dv_tutorial/bin/data/hill02.jpg");
    if (image1.empty() || image2.empty()) return -1;
    //convert to gray scale new image
    cv::Mat gray1, gray2;
    cv::cvtColor(image1, gray1, cv::COLOR_RGB2GRAY);
    cv::cvtColor(image2, gray2, cv::COLOR_RGB2GRAY);

    // Retrieve matching points
    cv::Ptr<cv::FeatureDetector> fdetector = cv::SIFT::create();
    std::vector<cv::KeyPoint> keypoint1, keypoint2;
    cv::Mat descriptor1, descriptor2;
    //time detectAndCompute
    auto start = std::chrono::steady_clock::now();
    fdetector->detectAndCompute(gray1, cv::Mat(), keypoint1, descriptor1);
    //time detectAndCompute
    auto end = std::chrono::steady_clock::now();
    //calculate time in ms
    auto diff = end - start;
    std::cout << std::chrono::duration<double, std::milli>(diff).count() << " ms for detectAndCompute" << std::endl;
    //time detectAndCompute2
    auto start2 = std::chrono::steady_clock::now();
    fdetector->detectAndCompute(gray2, cv::Mat(), keypoint2, descriptor2);
    //time detectAndCompute2
    auto end2 = std::chrono::steady_clock::now();
    //calculate time in ms
    auto diff2 = end2 - start2;
    std::cout << std::chrono::duration<double, std::milli>(diff2).count() << " ms for detectAndCompute2" << std::endl;

    //number of keypoint
    std::cout << "number of keypoint1: " << keypoint1.size() << std::endl;
    std::cout << "number of keypoint2: " << keypoint2.size() << std::endl;

    //SIFT matching with RANSAC
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce");
    std::vector<cv::DMatch> match;
    //time match
    auto start3 = std::chrono::steady_clock::now();
    //match with RANSAC
    matcher->match(descriptor1, descriptor2, match);
    //time match
    auto end3 = std::chrono::steady_clock::now();
    //calculate time in ms
    auto diff3 = end3 - start3;
    std::cout << std::chrono::duration<double, std::milli>(diff3).count() << " ms for match" << std::endl;

    //number of match
    std::cout << "number of match: " << match.size() << std::endl;

    // Calculate planar homography and merge them
    std::vector<cv::Point2f> points1, points2;
    for (auto & i : match)
    {
        points1.push_back(keypoint1.at(i.queryIdx).pt);
        points2.push_back(keypoint2.at(i.trainIdx).pt);
    }
    cv::Mat inlier_mask;
    cv::Mat H = cv::findHomography(points2, points1, inlier_mask, cv::RANSAC);

    cv::Mat merged;
    cv::warpPerspective(image2, merged, H, cv::Size(image1.cols * 2, image1.rows));
    merged.colRange(0, image1.cols) = image1 * 1; // Copy

    // Show the merged image
    cv::Mat original, matched;
    cv::drawMatches(image1, keypoint1, image2, keypoint2, match, matched, cv::Scalar::all(-1), cv::Scalar::all(-1), inlier_mask); // Remove 'inlier_mask' if you want to show all putative matches
    cv::hconcat(image1, image2, original);
    cv::vconcat(original, matched, matched);
    cv::vconcat(matched, merged, merged);
    cv::imshow("3DV Tutorial: Image Stitching", merged);
    cv::waitKey(0);
    return 0;
}
