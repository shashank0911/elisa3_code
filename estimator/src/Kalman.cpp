#include "Kalman.h"

Kalman::Kalman() {

    Ak1 << 1.0, 0.0, 0.0,
           0.0, 1.0, 0.0,
           0.0, 0.0, 1.0;

    processNoiseVkMinus1 << 0.0, 0.0, 0.0;

    Qk << 1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0;

    Hk << 1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0;

    Rk << 0.1, 0.0, 0.0,
          0.0, 0.1, 0.0,
          0.0, 0.0, 0.1;

    sensorNoiseWk << 0.0, 0.0, 0.0;

    Pk1 << 0.1, 0.0, 0.0,
           0.0, 0.1, 0.0,
           0.0, 0.0, 0.1;
}

std::pair<Eigen::Vector3d, Eigen::Matrix3d> Kalman::srEKF(const Eigen::Vector3d& Zk, const Eigen::Vector3d& stateEstimateK, double dk) {

    Eigen::Matrix3d Pk = Ak1 * Pk1 * Ak1.transpose() + Qk;
    Eigen::Vector3d measurementResidualYk = Zk - ((Hk * stateEstimateK) + sensorNoiseWk);
    Eigen::Matrix3d Sk = Hk * Pk * Hk.transpose() + Rk;
    Eigen::Matrix3d Kk = Pk * Hk.transpose() * Sk.inverse();

    Eigen::Vector3d optimalStateEstimateK = stateEstimateK + (Kk * measurementResidualYk);
    Pk = Pk - (Kk * Hk * Pk);

    return std::make_pair(optimalStateEstimateK, Pk);
}

std::pair<Eigen::Vector3d, Eigen::Matrix3d> Kalman::mrEKF(const Eigen::Vector3d& Zkm, const Eigen::Vector3d& stateEstimateKm, double dk) {

    Eigen::Matrix3d Pkm = Ak1 * Pk1 *Ak1.transpose() + 5 * Qk;
    Eigen::Vector3d measurementResidualYk = Zkm - ((Hk * stateEstimateKm) + sensorNoiseWk);
    Eigen::Matrix3d Skm = Hk * Pkm * Hk.transpose() + Rk;
    Eigen::Matrix3d Kkm = Pkm * Hk.transpose() * Skm.inverse();
    Eigen::Vector3d optimalStateEstimateKm = stateEstimateKm + (Kkm * measurementResidualYk);
    Pkm = Pkm - (Kkm * Hk * Pkm);

    return std::make_pair(optimalStateEstimateKm, Pkm);

}