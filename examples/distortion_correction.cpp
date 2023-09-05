#include "opencv2/opencv.hpp"

bool USE_REMAP = false;

int main()
{
    const char* input = "/home/dinhnambkhn/3dv_tutorial/bin/data/chessboard.avi";
    cv::Matx33d K(432.7390364738057, 0, 476.0614994349778, 0, 431.2395555913084, 288.7602152621297, 0, 0, 1);
    std::vector<double> dist_coeff = { -0.2852754904152874, 0.1016466459919075, -0.0004420196146339175, 0.0001149909868437517, -0.01803978785585194 };

    // Open a video
    cv::VideoCapture video;
    if (!video.open(input)) return -1;

    // Run distortion correction
    bool show_rectify = true;
    cv::Mat map1, map2;
    while (true)
    {

        // Grab an image from the video
        cv::Mat image;
        video >> image;
        if (image.empty()) break;
        //show image
        //cv::imshow("un-distortion", image);

        // Rectify geometric distortion (c.f. 'cv::undistort()' can be applied for one-time remapping.)
        //clone image to img_original
        cv::Mat img_original = image.clone();

        cv::String info = "Original";
        //put text on image
        cv::putText(img_original, info, cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, 1, cv::Vec3b(0, 255, 0));

        if (show_rectify)
        {
            //The initUndistortRectifyMap is simply a combination of initUndistortRectifyMap (with unity R ) and remap (with bilinear interpolation).
            // See the former function for details of the transformation being performed.
            if (map1.empty() || map2.empty())
                cv::initUndistortRectifyMap(K, dist_coeff, cv::Mat(), cv::Mat(), image.size(), CV_32FC1, map1, map2);
            cv::remap(image, image, map1, map2, cv::InterpolationFlags::INTER_LINEAR);
            info = "Rectified - map";
        }
        cv::putText(image, info, cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, 1, cv::Vec3b(0, 255, 0));

        //undistort image
        info = "Rectified - undistort";
        cv::Mat image_undistort;
        cv::undistort(img_original, image_undistort, K, dist_coeff);
        cv::putText(image_undistort, info, cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, 1, cv::Vec3b(0, 255, 0));

        //print size of image
        std::cout << "image size: " << image.size() << std::endl;
        std::cout << "image_undistort size: " << image_undistort.size() << std::endl;

        //copy image to img_rectify
        cv::Mat img_rectify = image.clone();
        // Show the image
        //cv::imshow("3DV Tutorial: Distortion Correction", image);
        //show original image on the left and rectified image on the right
        cv::Mat img_show;
        if(USE_REMAP)
            cv::hconcat(img_original, image, img_show);
        else
            cv::hconcat(img_original, image_undistort, img_show);

        cv::imshow("3DV Tutorial: Distortion Correction", img_show);

        int key = cv::waitKey(1);
        if (key == 27) break;                                   // 'ESC' key: Exit
        else if (key == 9) show_rectify = !show_rectify;        // 'Tab' key: Toggle rectification
        else if (key == 32)                                     // 'Space' key: Pause
        {
            key = cv::waitKey();
            if (key == 27) break;                               // 'ESC' key: Exit
            else if (key == 9) show_rectify = !show_rectify;    // 'Tab' key: Toggle rectification
        }
    }

    video.release();
    return 0;
}
