#include "Kalman.h"

Kalman::Kalman() {

    A_k_1 = Eigen::MatrixXd(3, 3);
    A_k_1 << 1.0, 0, 0,
             0, 1.0, 0,
             0, 0, 1.0;

    process_noise_v_k_minus_1 = Eigen::VectorXd(3);
    process_noise_v_k_minus_1 << 0.00, 0.00, 0.00;

    Q_k = Eigen::MatrixXd(3, 3);
    Q_k << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;

    H_k = Eigen::MatrixXd(3, 3);
    H_k << 1.0, 0, 0,
           0, 1.0, 0,
           0, 0, 1.0;

    R_k = Eigen::MatrixXd(3, 3);
    R_k << 0.1, 0, 0,
           0, 0.1, 0,
           0, 0, 0.1;

    sensor_noise_w_k = Eigen::VectorXd(3);
    sensor_noise_w_k << 0.00, 0.00, 0.00;

    P_k_1 = Eigen::MatrixXd(3, 3);
    P_k_1 << 0.1, 0, 0,
             0, 0.1, 0,
             0, 0, 0.1;
}

std::tuple<Eigen::VectorXd, Eigen::MatrixXd> Kalman::sr_EKF(const Eigen::VectorXd& z_k, const Eigen::VectorXd& state_estimate_k, double dk) {

    Eigen::MatrixXd P_k = this->A_k_1 * this->P_k_1 * this->A_k_1.transpose() + this->Q_k;
    Eigen::VectorXd measurement_residual_y_k = z_k - ((this->H_k * state_estimate_k) + this->sensor_noise_w_k);
    Eigen::MatrixXd S_k = this->H_k * P_k * this->H_k.transpose() + this->R_k;
    Eigen::MatrixXd K_k = P_k * this->H_k.transpose() * S_k.inverse();

    Eigen::VectorXd optimal_state_estimate_k = state_estimate_k * (K_k * measurement_residual_y_k);
    P_k = P_k - (K_k * this->H_k * P_k);

    return std::make_tuple(optimal_state_estimate_k, P_k);
}

Eigen::MatrixXd Kalman::get_B_M(int t) {

    Eigen::MatrixXd B_M = this->H_k;

    for (int i = 0; i < t-1; i++) {
        B_M.conservativeResize(B_M.rows(), B_M.cols() + this->H_k.cols());
        B_M.block(0, B_M.cols() - this->H_k.cols(), this->H_k.rows(), this->H_k.cols()) = this->H_k;
    }

    return B_M;
}

std::tuple<Eigen::VectorXd, Eigen::MatrixXd> Kalman::mr_EKF(const Eigen::VectorXd& z_k_M, const Eigen::VectorXd& state_estimate_k_M, const Eigen::VectorXd& u_k_1, double dk) {

    Eigen::MatrixXd B_M = this->get_B_M(5);

    Eigen::MatrixXd P_k_M = this->A_k_1 * this->P_k_1 *this->A_k_1.transpose() + 5 * this->Q_k;
    Eigen::VectorXd measurement_residual_y_k = z_k_M - ((this->H_k * state_estimate_k_M) + this->sensor_noise_w_k);
    Eigen::MatrixXd S_k_M = this->H_k * P_k_M * this->H_k.transpose() + this->R_k;
    Eigen::MatrixXd K_k_M = P_k_M * this->H_k.transpose() * S_k_M.inverse();
    Eigen::VectorXd optimal_state_estimate_k_M = state_estimate_k_M + (K_k_M * measurement_residual_y_k);
    P_k_M = P_k_M - (K_k_M * this->H_k * P_k_M);

    return std::make_tuple(optimal_state_estimate_k_M, P_k_M);

}