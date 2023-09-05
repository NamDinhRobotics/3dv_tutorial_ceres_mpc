#include "opencv2/opencv.hpp"

int main()
{
    // c.f. You need to run 'image_formation.cpp' to generate point observation.
    const char *input0 = "/home/dinhnambkhn/3dv_tutorial/bin/data/image_formation0.xyz", *input1 = "/home/dinhnambkhn/3dv_tutorial/bin/data/image_formation1.xyz";
    double f = 1000, cx = 320, cy = 240;

    // Load 2D points observed from two views
    std::vector<cv::Point2d> points0, points1;
    FILE* fin0 = fopen(input0, "rt");
    FILE* fin1 = fopen(input1, "rt");
    if (fin0 == nullptr || fin1 == nullptr) return -1;
    while (!feof(fin0) || !feof(fin1))
    {
        double x, y, w;
        if (!feof(fin0) && fscanf(fin0, "%lf %lf %lf", &x, &y, &w) == 3)
            points0.emplace_back(x, y);
        if (!feof(fin1) && fscanf(fin1, "%lf %lf %lf", &x, &y, &w) == 3)
            points1.emplace_back(x, y);
    }
    fclose(fin0);
    fclose(fin1);
    if (points0.size() != points1.size()) return -1;

    // Estimate relative pose of two views
    /* //using Essential Matrix - 5pt algorithm with RANSAC
    cv::Mat E, inlier_mask;
    E = cv::findEssentialMat(points0, points1, f, cv::Point2d(cx, cy), cv::RANSAC, 0.999, 1.0, inlier_mask);
    cv::Mat R, t;
    cv::recoverPose(E, points0, points1, R, t, f, cv::Point2d(cx, cy), inlier_mask);
    //K
    cv::Mat K = (cv::Mat_<double>(3, 3) << f, 0, cx, 0, f, cy, 0, 0, 1);
    */

    cv::Mat F = cv::findFundamentalMat(points0, points1, cv::FM_8POINT);
    cv::Mat K = (cv::Mat_<double>(3, 3) << f, 0, cx, 0, f, cy, 0, 0, 1);
    cv::Mat E = K.t() * F * K;
    cv::Mat R, t;
    cv::recoverPose(E, points0, points1, K, R, t);

    // Reconstruct 3D points (triangulation)
    cv::Mat P0 = K * cv::Mat::eye(3, 4, CV_64F);
    cv::Mat Rt, X;
    cv::hconcat(R, t, Rt);
    cv::Mat P1 = K * Rt;
    cv::triangulatePoints(P0, P1, points0, points1, X);
    X.row(0) = X.row(0) / X.row(3);
    X.row(1) = X.row(1) / X.row(3);
    X.row(2) = X.row(2) / X.row(3);
    X.row(3) = 1;

    // Store the 3D points
    FILE* fout = fopen("/home/dinhnambkhn/3dv_tutorial/bin/data/triangulation.xyz", "wt");
    if (fout == nullptr) return -1;
    for (int c = 0; c < X.cols; c++)
        fprintf(fout, "%f %f %f\n", X.at<double>(0, c), X.at<double>(1, c), X.at<double>(2, c));
    fclose(fout);
    return 0;
}
