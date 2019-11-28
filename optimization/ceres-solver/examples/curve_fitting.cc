// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: sameeragarwal@google.com (Sameer Agarwal)

#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

// Data generated using the following octave code.
//   randn('seed', 23497);
//   m = 0.3;
//   c = 0.1;
//   x=[0:0.075:5];
//   y = exp(m * x + c);
//   noise = randn(size(x)) * 0.2;
//   y_observed = y + noise;
//   data = [x', y_observed'];

const int kNumObservations = 67;
const double data[] = {
        0.000000e+00, 1.133898e+00,
        7.500000e-02, 1.334902e+00,
        1.500000e-01, 1.213546e+00,
        2.250000e-01, 1.252016e+00,
        3.000000e-01, 1.392265e+00,
        3.750000e-01, 1.314458e+00,
        4.500000e-01, 1.472541e+00,
        5.250000e-01, 1.536218e+00,
        6.000000e-01, 1.355679e+00,
        6.750000e-01, 1.463566e+00,
        7.500000e-01, 1.490201e+00,
        8.250000e-01, 1.658699e+00,
        9.000000e-01, 1.067574e+00,
        9.750000e-01, 1.464629e+00,
        1.050000e+00, 1.402653e+00,
        1.125000e+00, 1.713141e+00,
        1.200000e+00, 1.527021e+00,
        1.275000e+00, 1.702632e+00,
        1.350000e+00, 1.423899e+00,
        1.425000e+00, 1.543078e+00,
        1.500000e+00, 1.664015e+00,
        1.575000e+00, 1.732484e+00,
        1.650000e+00, 1.543296e+00,
        1.725000e+00, 1.959523e+00,
        1.800000e+00, 1.685132e+00,
        1.875000e+00, 1.951791e+00,
        1.950000e+00, 2.095346e+00,
        2.025000e+00, 2.361460e+00,
        2.100000e+00, 2.169119e+00,
        2.175000e+00, 2.061745e+00,
        2.250000e+00, 2.178641e+00,
        2.325000e+00, 2.104346e+00,
        2.400000e+00, 2.584470e+00,
        2.475000e+00, 1.914158e+00,
        2.550000e+00, 2.368375e+00,
        2.625000e+00, 2.686125e+00,
        2.700000e+00, 2.712395e+00,
        2.775000e+00, 2.499511e+00,
        2.850000e+00, 2.558897e+00,
        2.925000e+00, 2.309154e+00,
        3.000000e+00, 2.869503e+00,
        3.075000e+00, 3.116645e+00,
        3.150000e+00, 3.094907e+00,
        3.225000e+00, 2.471759e+00,
        3.300000e+00, 3.017131e+00,
        3.375000e+00, 3.232381e+00,
        3.450000e+00, 2.944596e+00,
        3.525000e+00, 3.385343e+00,
        3.600000e+00, 3.199826e+00,
        3.675000e+00, 3.423039e+00,
        3.750000e+00, 3.621552e+00,
        3.825000e+00, 3.559255e+00,
        3.900000e+00, 3.530713e+00,
        3.975000e+00, 3.561766e+00,
        4.050000e+00, 3.544574e+00,
        4.125000e+00, 3.867945e+00,
        4.200000e+00, 4.049776e+00,
        4.275000e+00, 3.885601e+00,
        4.350000e+00, 4.110505e+00,
        4.425000e+00, 4.345320e+00,
        4.500000e+00, 4.161241e+00,
        4.575000e+00, 4.363407e+00,
        4.650000e+00, 4.161576e+00,
        4.725000e+00, 4.619728e+00,
        4.800000e+00, 4.737410e+00,
        4.875000e+00, 4.727863e+00,
        4.950000e+00, 4.669206e+00,
};

//-------------------------------------------
// 自动求导
//-------------------------------------------
struct ExponentialResidual {
    ExponentialResidual(double x, double y)
            : x_(x), y_(y) {}
    // ------------【正确写法】------------
    template<typename T>
    bool operator()(const T *const m,
                    const T *const c,
                    T *residual) const {
        residual[0] = y_ - exp(m[0]*x_ + c[0]);
        return true;
    }
    // ------------【错误写法】------------
//    template<typename T>
//    bool operator()(const T *const x,
//                    T *residual) const {
//        residual[0] = y_ - exp(x[0]*x_ + x[1]);
//        return true;
//    }

private:
    const double x_;
    const double y_;
};


//-------------------------------------------
// 解析求导方式1
//-------------------------------------------
class AnalyticCostFunction
        : public ceres::SizedCostFunction<1 /* number of residuals */,
                2 /* size of first parameter */> {
public:
    AnalyticCostFunction(double x, double y) : x_(x), y_(y) {}
    virtual ~AnalyticCostFunction() {}

    /**
     * @brief 重载Evaluate函数，完成jacobian和residuals的计算
     * @param parameters
     * @param residuals
     * @param jacobians
     * @return
     */
    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const {
        double m = parameters[0][0]; // parameters[0]表示取出第一组参数
        double c = parameters[0][1];

        // 计算残差
        residuals[0] = y_ - exp(m*x_ + c);

        // 计算雅克比
        if (jacobians != NULL && jacobians[0] != NULL) {
            jacobians[0][0] = -x_*exp(m*x_ + c);
            jacobians[0][1] = -exp(m*x_ + c);
        }

        return true;
    }

private:
    const double x_;
    const double y_;
};


//-------------------------------------------
// 解析求导方式2
//-------------------------------------------
class AnalyticCostFunction2
        : public ceres::CostFunction {
public:
    AnalyticCostFunction2(const int &num_residuals, const int &block_sizes,
                          double x, double y) : x_(x), y_(y) {
        set_num_residuals(num_residuals); // 设置残差维度
        std::vector<int> *param_block_sizes = mutable_parameter_block_sizes(); // 返回的引用
        param_block_sizes->push_back(block_sizes); // 设置参数块维度，有几个参数块，就pubsh_back几次
    }
    virtual ~AnalyticCostFunction2() {}

    /**
     * @brief 重载Evaluate函数，完成jacobian和residuals的计算
     * @param parameters
     * @param residuals
     * @param jacobians
     * @return
     */
    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const {
        double m = parameters[0][0]; // parameters[0]表示取出第一组参数
        double c = parameters[0][1];

        // 计算残差
        residuals[0] = y_ - exp(m*x_ + c);

        // 计算雅克比
        if (jacobians != NULL && jacobians[0] != NULL) {
            jacobians[0][0] = -x_*exp(m*x_ + c);
            jacobians[0][1] = -exp(m*x_ + c);
        }

        return true;
    }

private:
    const double x_;
    const double y_;
};


//-------------------------------------------
// 数值求导
//-------------------------------------------
struct NumericalCostFunction {
    NumericalCostFunction(double x, double y)
            : x_(x), y_(y) {}
    // ------------【正确写法】------------
    bool operator()(const double *const m,
                    const double *const c,
                    double *residual) const {
        residual[0] = y_ - exp(m[0]*x_ + c[0]);
        return true;
    }

private:
    const double x_;
    const double y_;
};

//--------------------------------------------------
// GradientChecker: Jacobians检查工具
// TODO: 程序不对，留坑
// ref: http://ceres-solver.org/nnls_modeling.html#gradientchecker
//--------------------------------------------------
void GradientCheckerTest(CostFunction *my_cost_function,
                         std::vector<double> &parameter,
                         ceres::LocalParameterization *my_parameterization = nullptr) {
    ceres::NumericDiffOptions numeric_diff_options;

    std::vector<const ceres::LocalParameterization *> local_parameterizations;
    local_parameterizations.push_back(my_parameterization);

    std::vector<double *> parameter_blocks;
    parameter_blocks.push_back(parameter.data());
    ceres::GradientChecker gradient_checker(my_cost_function, &local_parameterizations, numeric_diff_options);
    ceres::GradientChecker::ProbeResults results;
    if (!gradient_checker.Probe(parameter_blocks.data(), 1e-9, &results)) {
        LOG(ERROR) << "An error has occurred:\n" << results.error_log;
    }
}

#define Numerical
//        AutoDiff = 1,
//        Analytic = 2,
//        Numerical = 3
int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    Problem problem;

#ifdef Analytic
    double x[2] = {0, 0};
    for (int i = 0; i < kNumObservations; ++i) {
        CostFunction *cost_function = new AnalyticCostFunction2(1,2,data[2*i], data[2*i + 1]);
        problem.AddResidualBlock(cost_function, NULL, x);
    }
    Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "---------CHOSE_ANALYTIC_DIFF---------\n";
    std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
    std::cout << "Final   m: " << x[0] << " c: " << x[1] << "\n";
#endif

#ifdef AutoDiff
    double m = 0.0;
    double c = 0.0;

    double x[2] = {0, 0};
    for (int i = 0; i < kNumObservations; ++i) {
        // ------------【正确写法】------------
        problem.AddResidualBlock(
                new AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>(
                        new ExponentialResidual(data[2*i], data[2*i + 1])),
                NULL,
                &m, &c);
        // ------------【错误写法】------------
//        problem.AddResidualBlock(
//                new AutoDiffCostFunction<ExponentialResidual, 1, 2>(
//                        new ExponentialResidual(data[2*i], data[2*i + 1])),
//                NULL,
//                x);
    }
    Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
    std::cout << "Final   m: " << m << " c: " << c << "\n";
#endif

#ifdef Numerical
    double m = 0.0;
    double c = 0.0;

    double x[2] = {0, 0};
    for (int i = 0; i < kNumObservations; ++i) {
        ceres::CostFunction *my_cost_function = new ceres::NumericDiffCostFunction<NumericalCostFunction, ceres::RIDDERS, 1, 1, 1>(
                new NumericalCostFunction(data[2*i], data[2*i + 1]));
        problem.AddResidualBlock(my_cost_function, NULL, &m, &c);
    }
    Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
    std::cout << "Final   m: " << m << " c: " << c << "\n";
#endif


    return 0;
}
