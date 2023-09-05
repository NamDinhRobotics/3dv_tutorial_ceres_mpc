//
// Created by dinhnambkhn on 28/08/2023.
//
#include <iostream>
#include <ceres/ceres.h>

//create a functor, drive from 0 to step k that reach the goal with data type T
struct KineticModelFunctor {
    //constructor
    KineticModelFunctor(const double* start, const double* goal) {
        //copy start and goal state
        for (int i = 0; i < 5; i++) {
            this->start[i] = start[i];
            this->goal[i] = goal[i];
        }
    }
    //operator which calculate residual = (x_k+1 - x_goal)^2 + (y_k+1 - y_goal)^2 + (theta_k+1 - theta_goal)^2
    template<typename T>
    bool operator()(const T* const control_input, T* residual) const {
        //create a state with start state
        T* state = new T[5];
        for (int i = 0; i < 5; i++) {
            state[i] = T(start[i]);
        }
        //sum of square difference between current and previous control input
        T sum = T(0.0);
        //propagate kinetic model from 0 to step k
        for (int i = 0; i < 10; i++) {
            //propagate kinetic model at step k+1
            //control input = [v, w] with dt = 0.1, xk+1
            T control_temp[2] = {control_input[2*i], control_input[2*i + 1]};
            //xk+1 = xk + v*cos(theta)*dt
            state[0] += control_temp[0] * cos(state[2]) * 0.1;
            //yk+1 = yk + v*sin(theta)*dt
            state[1] += control_temp[0] * sin(state[2]) * 0.1;
            //thetak+1 = thetak + w*dt
            state[2] += control_temp[1] * 0.1;
            //vk+1 = v
            state[3] = control_temp[0];
            //wk+1 = w
            state[4] = control_temp[1];

            //calculate sum of square difference between current and previous control input
            sum += (control_temp[0] - control_input[2*i]) * (control_temp[0] - control_input[2*i]) + (control_temp[1] - control_input[2*i + 1]) * (control_temp[1] - control_input[2*i + 1]);
        }
        //calculate residual to reach goal state
        residual[0] = T(0.1)* (state[0] - T(goal[0]));
        residual[1] = T(0.1)* (state[1] - T(goal[1]));
        residual[2] = T(0.1)* (state[2] - T(goal[2]));
        // TODO: minimize the angular acceleration
        residual[3] = T(1.0) * sum;

        //residual[3] = state[3] - T(goal[3]);
        //residual[4] = state[4] - T(goal[4]);

        return true;
    }
    //create a cost function with 20 DOF control input and 3 DOF residual
    static ceres::CostFunction* create(const double* start, const double* goal) {
        return (new ceres::AutoDiffCostFunction<KineticModelFunctor, 4, 20>(new KineticModelFunctor(start, goal)));
    }


private:
    //start state
    double start[5]{};
    //goal state
    double goal[5]{};

};

int main()
{
    //generate start state and goal state
    double start[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double goal[5] = {1.5, 0.5, 0.04, 0.0, 0.0};

    //control input with 20 values 0.0
    double control_input[20];
    for (double & i : control_input) {
        i = 0.0;
    }
    //create ceres problem
    ceres::Problem problem;
    //create cost function
    ceres::CostFunction* cost_function = KineticModelFunctor::create(start, goal);
    //add residual block to problem
    problem.AddResidualBlock(cost_function, nullptr, control_input);

    //set solver options
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    //add constraints to control input
    for (int i = 0; i < 10; i++) {
        //control input = [v, w] with dt = 0.1
        //v in range [0.0, 1.0]
        problem.SetParameterLowerBound(control_input, 2*i, 0.0);
        problem.SetParameterUpperBound(control_input, 2*i, 21.0);
        //w in range [-0.5, 0.5]
        problem.SetParameterLowerBound(control_input, 2*i + 1, -1.0);
        problem.SetParameterUpperBound(control_input, 2*i + 1, 1.0);
    }
    //solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
    //print control input
    std::cout << "control input: " << std::endl;
    for (double i : control_input) {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    //print state propagation
    std::cout << "state: " << std::endl;

    double state[5];
    for (int i = 0; i < 5; i++) {
        state[i] = start[i];
    }
    for (int i = 0; i < 10; i++) {
        state[0] += control_input[i] * cos(state[2]) * 0.1;
        state[1] += control_input[i] * sin(state[2]) * 0.1;
        state[2] += control_input[i + 1] * 0.1;
        state[3] = control_input[i];
        state[4] = control_input[i + 1];
    }

    for (double j : state) {
        std::cout << j << " ";
    }
    std::cout << std::endl;

    //sum of error steering
    std::cout << "error steering: " << std::endl;
    double sum = 0.0;
    for (int i = 0; i < 10; i++) {
        sum += (control_input[2*i] - control_input[2*i + 2]) * (control_input[2*i] - control_input[2*i + 2]) + (control_input[2*i + 1] - control_input[2*i + 3]) * (control_input[2*i + 1] - control_input[2*i + 3]);
    }
    std::cout << sum << std::endl;


    return 0;

}