#include <thread>
#include "opencv2/opencv.hpp"


int main() {
    const char *input = "/home/dinhnambkhn/3dv_tutorial/bin/data/07/image_0/%06d.png";
    double f = 707.0912;
    cv::Point2d c(601.8873, 183.1104);
    bool use_5pt = true;
    int min_inlier_num = 100;

    // Open a file to write camera trajectory
    FILE *camera_traj = fopen("/home/dinhnambkhn/3dv_tutorial/bin/data/vo_epipolar2.xyz", "wt");
    if (camera_traj == nullptr) return -1;

    // Open a video and get the initial image
    cv::VideoCapture video;
    if (!video.open(input)) return -1;

    cv::Mat gray_prev;
    video >> gray_prev;
    if (gray_prev.empty()) {
        video.release();
        return -1;
    }
    if (gray_prev.channels() > 1) cv::cvtColor(gray_prev, gray_prev, cv::COLOR_RGB2GRAY);

    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);

    // Run and record monocular visual odometry
    cv::Mat camera_pose = cv::Mat::eye(4, 4, CV_64F);
    while (true) {
        //start time
        auto start = std::chrono::steady_clock::now();
        // Grab an image from the video
        cv::Mat image, gray;
        video >> image;
        if (image.empty()) break;
        if (image.channels() > 1) cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
        else gray = image.clone();

        // Extract optical flow
        std::vector<cv::Point2f> point_prev, point;
        cv::goodFeaturesToTrack(gray_prev, point_prev, 2000, 0.01, 10);
        //time extract optical flow
        auto end1 = std::chrono::steady_clock::now();
        //calculate time in ms
        auto diff1 = end1 - start;
        std::cout << std::chrono::duration<double, std::milli>(diff1).count() << " ms for GFTT" << std::endl;

        //extract FAST
        std::vector<cv::KeyPoint> point_prev_fast;
        cv::FAST(gray_prev, point_prev_fast, 10, true);
        //time extract FAST - test what time-consuming
        auto end3 = std::chrono::steady_clock::now();
        //calculate time in ms
        auto diff3 = end3 - end1;
        std::cout << std::chrono::duration<double, std::milli>(diff3).count() << " ms for FAST" << std::endl;
        //extract ORB
        std::vector<cv::KeyPoint> point_prev_orb;
        orb->detect(gray_prev, point_prev_orb);
        //time extract ORB
        auto end4 = std::chrono::steady_clock::now();
        //calculate time in ms
        auto diff4 = end4 - end3;
        std::cout << std::chrono::duration<double, std::milli>(diff4).count() << " ms for ORB" << std::endl;

        //detect line and precision
        std::vector<cv::Vec4i> lines;
        cv::Mat pres;
        ls->detect(gray_prev, lines, cv::noArray(), pres, cv::noArray());

        //time detect line
        auto end5 = std::chrono::steady_clock::now();
        //calculate time in ms
        auto diff5 = end5 - end4;
        std::cout << std::chrono::duration<double, std::milli>(diff5).count() << " ms for LSD" << std::endl;

        //only select 20 lines with high confidence pres
        std::vector<cv::Vec4i> lines_select;
        for (int i = 0; i < lines.size(); i++) {
            if (pres.at<float>(i) > 0.99) {
                lines_select.push_back(lines[i]);
            }
            //if (lines_select.size() > 20) break;
        }

        //show the line on image by yellow
        ls->drawSegments(image, lines_select);
        //imshow on new window
       // cv::imshow("3DV Tutorial: Visual Odometry (Epipolar)", image);


        std::vector<uchar> status;
        cv::Mat err;
        cv::calcOpticalFlowPyrLK(gray_prev, gray, point_prev, point, status, err);
        //time calcOpticalFlowPyrLK
        auto end2 = std::chrono::steady_clock::now();
        //calculate time in ms
        auto diff2 = end2 - end1;
        std::cout << std::chrono::duration<double, std::milli>(diff2).count() << " ms for calcOpticalFlowPyrLK" << std::endl;

        gray_prev = gray;

        // Calculate relative pose
        cv::Mat E, inlier_mask;
        if (use_5pt) {
            E = cv::findEssentialMat(point_prev, point, f, c, cv::RANSAC, 0.99, 1, inlier_mask);
        } else {
            cv::Mat F = cv::findFundamentalMat(point_prev, point, cv::FM_RANSAC, 1, 0.99, inlier_mask);
            cv::Mat K = (cv::Mat_<double>(3, 3) << f, 0, c.x, 0, f, c.y, 0, 0, 1);
            E = K.t() * F * K;
        }
        cv::Mat R, t;
        int inlier_num = cv::recoverPose(E, point_prev, point, R, t, f, c, inlier_mask);

        // Accumulate relative pose if result is reliable
        if (inlier_num > min_inlier_num) {
            cv::Mat T = cv::Mat::eye(4, 4, R.type());
            T(cv::Rect(0, 0, 3, 3)) = R * 1.0;
            T.col(3).rowRange(0, 3) = t * 1.0;
            camera_pose = camera_pose * T.inv();
        }

        // Show the image and write camera pose 
        if (image.channels() < 3) cv::cvtColor(image, image, cv::COLOR_GRAY2RGB);
        for (int i = 0; i < point_prev.size(); i++) {
            if (inlier_mask.at<uchar>(i) > 0) cv::line(image, point_prev[i], point[i], cv::Vec3b(255, 0, 255)); //green
            //else cv::line(image, point_prev[i], point[i], cv::Vec3b(0, 127, 0));
            //else by yellow
            else cv::line(image, point_prev[i], point[i], cv::Vec3b(0, 255, 255));
        }
        cv::String info = cv::format("Inliers: %d (%zu%%),  XYZ: [%.3f, %.3f, %.3f]", inlier_num,
                                     100 * inlier_num / point.size(), camera_pose.at<double>(0, 3),
                                     camera_pose.at<double>(1, 3), camera_pose.at<double>(2, 3));
        cv::putText(image, info, cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, 1, cv::Vec3b(0, 255, 0));
        cv::imshow("3DV Tutorial: Visual Odometry (Epipolar)", image);
        fprintf(camera_traj, "%.6f %.6f %.6f\n", camera_pose.at<double>(0, 3), camera_pose.at<double>(1, 3),
                camera_pose.at<double>(2, 3));
        if (cv::waitKey(1) == 27) break; // 'ESC' key: Exit
        //end time
        auto end = std::chrono::steady_clock::now();
        //calculate time in ms
        auto diff = end - start;
        std::cout << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;
        //sleep for 300ms chrono
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    video.release();
    fclose(camera_traj);
    return 0;
}
