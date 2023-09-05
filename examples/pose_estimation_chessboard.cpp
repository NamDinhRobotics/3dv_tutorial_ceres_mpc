#include <Eigen/Core>
#include "opencv2/opencv.hpp"

int main()
{
    const char* input = "/home/dinhnambkhn/3dv_tutorial/bin/data/chessboard.avi";
    cv::Matx33d K(432.7390364738057, 0, 476.0614994349778, 0, 431.2395555913084, 288.7602152621297, 0, 0, 1);
    std::vector<double> dist_coeff = { -0.2852754904152874, 0.1016466459919075, -0.0004420196146339175, 0.0001149909868437517, -0.01803978785585194 };
    cv::Size board_pattern(10, 7);
    double board_cellsize = 0.025;

    // Open a video
    cv::VideoCapture video;
    if (!video.open(input)) return -1;

    // Prepare a 3D box for simple AR
    std::vector<cv::Point3d> box_lower = { cv::Point3d(4 * board_cellsize, 2 * board_cellsize, 0), cv::Point3d(5 * board_cellsize, 2 * board_cellsize, 0), cv::Point3d(5 * board_cellsize, 4 * board_cellsize, 0), cv::Point3d(4 * board_cellsize, 4 * board_cellsize, 0) };
    std::vector<cv::Point3d> box_upper = { cv::Point3d(4 * board_cellsize, 2 * board_cellsize, -board_cellsize), cv::Point3d(5 * board_cellsize, 2 * board_cellsize, -board_cellsize), cv::Point3d(5 * board_cellsize, 4 * board_cellsize, -board_cellsize), cv::Point3d(4 * board_cellsize, 4 * board_cellsize, -board_cellsize) };

    // Prepare 3D points on a chessboard
    std::vector<cv::Point3d> obj_points;
    for (int r = 0; r < board_pattern.height; r++)
        for (int c = 0; c < board_pattern.width; c++)
            obj_points.emplace_back(board_cellsize * c, board_cellsize * r, 0);

    // Run pose estimation
    while (true)
    {
        // Grab an image from the video
        cv::Mat image;
        video >> image;
        if (image.empty()) break;

        // Estimate camera pose
        std::vector<cv::Point2d> img_points;
        bool success = cv::findChessboardCorners(image, board_pattern, img_points, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
        if (success)
        {
            cv::Mat rvec, tvec; // Rotation vector and translation vector from world to camera
            cv::solvePnP(obj_points, img_points, K, dist_coeff, rvec, tvec);

            //print tvec
            std::cout << "tvec = " << tvec << std::endl;
            //print rvec to roll pitch yaw
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            //R to eigen matrix
            Eigen::Matrix3d R_eigen;
            R_eigen << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
            //eigen matrix to yaw


            // Draw the box on the image
            cv::Mat line_lower, line_upper;
            // note: 	The rotation vector (Rodrigues) that, together with tvec, T_world_2_camera,
            // performs a change of basis from world to camera coordinate system, see calibrateCamera for details.
            cv::projectPoints(box_lower, rvec, tvec, K, dist_coeff, line_lower);
            cv::projectPoints(box_upper, rvec, tvec, K, dist_coeff, line_upper);
            line_lower.reshape(1).convertTo(line_lower, CV_32S); // Change 4 x 1 matrix (CV_64FC2) to 4 x 2 matrix (CV_32SC1)
            line_upper.reshape(1).convertTo(line_upper, CV_32S); // Because 'cv::polylines()' only accepts 'CV_32S' depth.
            cv::polylines(image, line_lower, true, cv::Vec3b(255, 0, 0), 2);
            for (int i = 0; i < line_lower.rows; i++)
                cv::line(image, cv::Point(line_lower.row(i)), cv::Point(line_upper.row(i)), cv::Vec3b(0, 255, 0), 2);
            cv::polylines(image, line_upper, true, cv::Vec3b(0, 0, 255), 2);

            // Print camera position in the world coordinate system
            // T_camera_2_world = T_world_2_camera^-1 = [R' -R'*tvec; 0 0 0 1] --> tvec = -R'*tvec
            cv::Mat R1;
            cv::Rodrigues(rvec, R1);
            cv::Mat p = -R1.t() * tvec; //tvec = -R'*tvec
            cv::String info = cv::format("tvec: [%.3f, %.3f, %.3f]; XYZ: [%.3f, %.3f, %.3f]",cv::Point3d(tvec).x,cv::Point3d(tvec).y,cv::Point3d(tvec).z,  cv::Point3d(p).x, cv::Point3d(p).y, cv::Point3d(p).z);
            cv::putText(image, info, cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, 1, cv::Vec3b(0, 255, 0));
        }

        // Show the image
        cv::imshow("3DV Tutorial: Pose Estimation (Chessboard)", image);
        int key = cv::waitKey(1);
        if (key == 27) break; // 'ESC' key: Exit
    }

    video.release();
    return 0;
}
