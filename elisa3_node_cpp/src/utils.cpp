#include "utils.h"

// Eigen::MatrixXd getDomain() {
//     Eigen::MatrixXd domain(4,2);
//     domain << 0, 0, 0, 1.15, 2.4, 1.15, 2.4, 0;
//     return domain;
// }

Eigen::Vector2d cart2pol(const Eigen::Vector2d& cart) {

    double rho = cart.norm();
    double phi = std::atan2(cart[1], cart[0]);
    return Eigen::Vector2d(rho, phi);
}

Eigen::Vector2d pol2cart (const Eigen::Vector2d& pol) {

    double x = pol[0] * std::cos(pol[1]);
    double y = pol[0] * std::sin(pol[1]);
    return Eigen::Vector2d(x, y);
}

Eigen::VectorXd renewVec(const Eigen::VectorXd& oldVec) {

    Eigen::VectorXd newVec(oldVec.size());

    if(oldVec.size() > 0) {
        newVec.head(oldVec.size() - 1) = oldVec.tail(oldVec.size() - 1);
        newVec(oldVec.size() - 1) = oldVec(oldVec.size() - 1);
    }

    return newVec;
}

double yawFromQuaternion(double x, double y, double z, double w) {
    double t3 = 2.0 * (w * z + x * y);
    double t4 = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(t3, t4);
}

ObstacleAvoidance::ObstacleAvoidance() {
    refLinesDomain = Eigen::MatrixXd(4,2);
    refLinesDomain << 0.0, 0.0, 
                        0.0, 1.15, 
                        2.4, 1.15, 
                        2.4, 0.0;
}

Eigen::Vector2d ObstacleAvoidance::perpendicular(const Eigen::Vector2d& a) {
    Eigen::Vector2d b;
    b << a[1], -a[0];
    return b;
}

double ObstacleAvoidance::det(const Eigen::Matrix2d& mat) {
    return (mat(0,0)*mat(1,1) - mat(0,1)*mat(1,0));
}

bool ObstacleAvoidance::checkDirectionVectors(const Eigen::Vector2d& vector1, const Eigen::Vector2d& vector2) {
    Eigen::Vector2d unitVector1 = vector1.normalized();
    Eigen::Vector2d unitVector2 = vector2.normalized();
    double dotProd = unitVector1.dot(unitVector2);
    return dotProd > 0;
}

bool ObstacleAvoidance::checkInDomain(const Eigen::Vector2d& point) {

    if(point[0] > this->refLinesDomain(0,0) && point[0] < this->refLinesDomain(2,0) 
    && point[1] > this->refLinesDomain(0,1) && point[1] < this->refLinesDomain(1,1)) {
        if(obstacles) {
            return true;
        } else {
            return false;
        }
    }
}

std::pair<int, Eigen::Vector2d> ObstacleAvoidance::lineIntersection(const Eigen::Matrix2d& locations) {

    std::map<int, std::tuple<Eigen::Vector2d, double>> potIter;
    Eigen::Vector2d moveVec = locations.col(1) - locations.col(0);

    for (int count = 0; count < this->refLinesDomain.rows(); count++) {
        int countPlus = 0;
        if (count + 1 > this->refLinesDomain.rows()) {
            countPlus = 0;
        } else {
            countPlus = count;
        }

        Eigen::Vector2d xdiff;
        xdiff(0) = this->refLinesDomain(count,0) - this->refLinesDomain(countPlus,0);
        xdiff(1) = locations(0,0) - locations(1,0);
        Eigen::Vector2d ydiff;
        ydiff(0) = this->refLinesDomain(count,1) - this->refLinesDomain(countPlus,1);
        ydiff(1) = locations(0,1) - locations(1,1); 
        Eigen::Matrix2d diff;
        diff << xdiff, ydiff;
        Eigen::Matrix2d refLineMinor;
        refLineMinor << this->refLinesDomain.row(count),
                          this->refLinesDomain.row(count+1);

        double div = this->det(diff);
        if (div != 0) {
            Eigen::Vector2d d;
            d << this->det(refLineMinor), this->det(locations);
            Eigen::Vector2d inter;
            inter << this->det(Eigen::Matrix2d(d, xdiff)) / div, this->det(Eigen::Matrix2d(d, ydiff)) / div;
            Eigen::Vector2d interVec = inter - locations.col(0);

            if (this->checkDirectionVectors(moveVec, interVec)) {
                potIter[count] = std::make_tuple(inter, interVec.norm());
            }
            
        }
    }

    if (!potIter.empty()) {
        auto keyMin = std::min_element(potIter.begin(), potIter.end(), 
            [](const auto& lhs, const auto& rhs) {
                return std::get<1>(lhs.second) < std::get<1>(rhs.second);
            });
        
        return std::make_pair(keyMin->first, std::get<0>(keyMin->second));
    } else {
        return std::make_pair(-1, Eigen::Vector2d::Zero());
    }
}

std::pair<Eigen::Vector2d, bool> ObstacleAvoidance::obstacleAvoidance(const Eigen::Vector2d& startPoint, const Eigen::Vector2d& move) {

    Eigen::Vector2d no_obs_newPoint = startPoint + move;

    if (!this->checkInDomain(startPoint + move)) {
        auto [indexLine, inter] = this->lineIntersection(Eigen::Matrix2d(startPoint, startPoint + move));
        if (indexLine != -1) {
            
        }
    }
}



