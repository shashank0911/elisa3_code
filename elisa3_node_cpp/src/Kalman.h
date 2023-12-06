#pragma once

#include <Eigen/Dense>
#include <tuple>

class Kalman {
    public:

        Kalman();

        std::tuple<Eigen::VectorXd, Eigen::MatrixXd> sr_EKF(const Eigen::VectorXd& z_k, const Eigen::VectorXd& state_estimate_k, double dk = 1);
    
        Eigen::MatrixXd get_B_M(int t);

        std::tuple<Eigen::VectorXd, Eigen::MatrixXd> mr_EKF(const Eigen::VectorXd& z_k_M, const Eigen::VectorXd& state_estimate_k_M, const Eigen::VectorXd& u_k_1, double dk = 1);

    private:
        Eigen::MatrixXd A_k_1;
        Eigen::VectorXd process_noise_v_k_minus_1;
        Eigen::MatrixXd Q_k;
        Eigen::MatrixXd H_k;
        Eigen::MatrixXd R_k;
        Eigen::VectorXd sensor_noise_w_k;
        Eigen::MatrixXd P_k_1;

};