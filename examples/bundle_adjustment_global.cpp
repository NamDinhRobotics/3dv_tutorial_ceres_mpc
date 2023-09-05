#include "bundle_adjustment.hpp"

int main()
{
    // c.f. The following dataset were generated by 'image_formation.cpp'.
    const char* input = "/home/dinhnambkhn/3dv_tutorial/bin/data/image_formation%d.xyz";
    int input_num = 5;
    double f = 1000, cx = 320, cy= 240;

    // Load 2D points observed from multiple views
    std::vector<std::vector<cv::Point2d>> xs;
    for (int i = 0; i < input_num; i++)
    {
        FILE* fin = fopen(cv::format(input, i).c_str(), "rt");
        if (fin == nullptr) return -1;
        std::vector<cv::Point2d> pts;
        while (!feof(fin))
        {
            double x, y, w;
            if (fscanf(fin, "%lf %lf %lf", &x, &y, &w) == 3)
                pts.emplace_back(x, y);
        }
        fclose(fin);
        xs.push_back(pts);
        if (xs.front().size() != xs.back().size()) return -1;
    }

    // Assumption
    // - All cameras have the same and known camera matrix.
    // - All points are visible on all camera views.

    // Initialize cameras and 3D points
    std::vector<cv::Vec6d> cameras(xs.size());
    std::vector<cv::Point3d> Xs(xs.front().size(), cv::Point3d(0, 0, 5.5));

    // Optimize camera pose and 3D points together (bundle adjustment)
    ceres::Problem ba;
    for (size_t j = 0; j < xs.size(); j++)
    {
        for (size_t i = 0; i < xs[j].size(); i++)
        {
            ceres::CostFunction* cost_func = ReprojectionError::create(xs[j][i], f, cv::Point2d(cx, cy));
            auto* camera = (double*)(&(cameras[j]));
            auto* X = (double*)(&(Xs[i]));
            ba.AddResidualBlock(cost_func, nullptr, camera, X);
        }
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.num_threads = 8;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &ba, &summary);

    // Store the 3D points to an XYZ file
    FILE* fpts = fopen("/home/dinhnambkhn/3dv_tutorial/bin/data/bundle_adjustment_global(point).xyz", "wt");
    if (fpts == nullptr) return -1;
    for (auto & X : Xs)
        fprintf(fpts, "%f %f %f\n", X.x, X.y, X.z); // Format: x, y, z
    fclose(fpts);

    // Store the camera poses to an XYZ file 
    FILE* fcam = fopen("/home/dinhnambkhn/3dv_tutorial/bin/data/bundle_adjustment_global(camera).xyz", "wt");
    if (fcam == nullptr) return -1;
    for (auto & camera : cameras)
    {
        cv::Vec3d rvec(camera[0], camera[1], camera[2]), t(camera[3], camera[4], camera[5]);
        cv::Matx33d R;
        cv::Rodrigues(rvec, R);
        cv::Vec3d p = -R.t() * t;
        fprintf(fcam, "%f %f %f %f %f %f\n", p[0], p[1], p[2], R.t()(0, 2), R.t()(1, 2), R.t()(2, 2)); // Format: x, y, z, n_x, n_y, n_z
    }
    fclose(fcam);
    return 0;
}