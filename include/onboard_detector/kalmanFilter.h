/*
	FILE: kalman_filter.h
	--------------------------------------
	header of kalman_filter velocity estimator
*/

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

using Eigen::MatrixXd;
using namespace std;

namespace onboardDetector{
    class kalman_filter
    {
        private:
        // 是否已经初始化
        bool is_initialized;
        // 状态矩阵
        MatrixXd states;
        // 状态转移矩阵
        MatrixXd A; // state matrix
        // 控制输入矩阵
        MatrixXd B; // input matrix
        // 观测矩阵
        MatrixXd H; // observation matrix
        // 协方差矩阵
        MatrixXd P; // uncertianty
        // 过程噪声协方差矩阵
        MatrixXd Q; // process noise
        // 观测噪声协方差矩阵
        MatrixXd R; // obsevation noise

        public:
        // constructor
        kalman_filter();

        // 设定滤波器参数
        void setup(const MatrixXd& states,
                   const MatrixXd& A,
                   const MatrixXd& B,
                   const MatrixXd& H,
                   const MatrixXd& P,
                   const MatrixXd& Q,
                   const MatrixXd& R);

        // set A (sometimes sampling time will differ)
        // 设定A矩阵
        void setA(const MatrixXd& A);

        // state estimate
        // 观测值,控制输入
        void estimate(const MatrixXd& z, const MatrixXd& u);

        // read output from the state
        // 输出估计状态
        double output(int state_index);
    };
}

#endif