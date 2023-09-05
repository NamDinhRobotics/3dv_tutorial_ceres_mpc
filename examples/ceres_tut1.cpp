//
// Created by dinhnambkhn on 27/08/2023.
//
#include <iostream>
#include <ceres/ceres.h>

//create a functor with residual = (10-x)^2+(3-y)^2
struct CostFunctor {
    template<typename T>
    bool operator()(const T *const x, T *residual) const {
        residual[0] = T(10.0) - x[0];
        residual[1] = T(3.0) - x[1];
        return true;
    }
};
int main() {
   double x[2] = {0.0, 1.0};
   double y[2] = {6.0, 5.0};

    ceres::Problem problem;
    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 2, 2>(new CostFunctor);
    problem.AddResidualBlock(cost_function, nullptr, x);
    problem.AddResidualBlock(cost_function, nullptr, y);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "x : " << x[0] << " " << x[1] << std::endl;
    std::cout << "y : " << y[0] << " " << y[1] << std::endl;
    return 0;
}