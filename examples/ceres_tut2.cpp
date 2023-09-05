//
// Created by dinhnambkhn on 27/08/2023.
//
#include <iostream>
#include <ceres/ceres.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

//create a curve fitting functor with residual = a*x^2+b*x+c-y, variable is a,b,c
struct CurveFittingFunctor {
    CurveFittingFunctor(double x, double y) : x_(x), y_(y) {}

    template<typename T>
    bool operator()(const T *const abc, T *residual) const {
        residual[0] = T(y_) - abc[0] * T(x_) * T(x_) - abc[1] * T(x_) - abc[2];
        return true;
    }
private:
    const double x_;
    const double y_;
};

int main(){
    //create dataset with all point inside image(480, 640) = (y, x): a*x^2+b*x+c--> a*320^2+b*320+c = 240
    double a = 1.0, b = 2.0, c = 1.0;


    int N = 100;
    double w_sigma = 0.2;
    cv::RNG rng;
    double abc[3] = {0.0, 0.0, 0.0};
    std::vector<double> x_data, y_data;
    std::cout << "generating data" << std::endl;
    for (int i = 0; i < N; i++) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(a * x * x + b * x + c + rng.gaussian(w_sigma));
    }
    std::cout << "generating data done" << std::endl;
    //create ceres problem
    ceres::Problem problem;
    for (int i = 0; i < N; i++) {
        ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CurveFittingFunctor, 1, 3>(
                new CurveFittingFunctor(x_data[i], y_data[i]));
        problem.AddResidualBlock(cost_function, nullptr, abc);
    }
    //set solver options
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    //solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "a : " << abc[0] << " b : " << abc[1] << " c : " << abc[2] << std::endl;

    //plot
    cv::Mat img(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < N; i++) {
        cv::circle(img, cv::Point(x_data[i] * 100, y_data[i] * 100), 1, cv::Scalar(0, 0, 255), 2);
    }
    for (int i = 0; i < 640; i++) {
        double x = i / 100.0;
        double y = abc[0] * x * x + abc[1] * x + abc[2];
        cv::circle(img, cv::Point(x * 100, y * 100), 1, cv::Scalar(255, 0, 0), 2);
    }
    cv::imshow("curve fitting", img);
    cv::waitKey(0);

    return 0;

}

