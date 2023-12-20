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

Eigen::MatrixXd renewVec(const Eigen::MatrixXd& oldVec) {

    Eigen::MatrixXd newVec(oldVec.rows(), oldVec.cols());

    if(oldVec.cols() > 0) {
        newVec.leftCols(oldVec.cols() - 1) = oldVec.rightCols(oldVec.cols() - 1);
        newVec.col(oldVec.cols() - 1) = oldVec.col(oldVec.cols() - 1);
    }

    return newVec;
}

double yawFromQuaternion(double x, double y, double z, double w) {
    double t3 = 2.0 * (w * z + x * y);
    double t4 = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(t3, t4);
}

ObstacleAvoidance::ObstacleAvoidance() {
    std::vector<Eigen::Matrix2d> refLinesDomain(4);
    // refLinesDomain << 0.0, 0.0, 
    //                   0.0, 1.15, 
    //                   2.4, 1.15, 
    //                   2.4, 0.0;
    refLinesDomain[0] << 0.0, 0.0, 0.0, 1.15;
    refLinesDomain[1] << 0.0, 1.15, 2.4, 1.15;
    refLinesDomain[2] << 2.4, 1.15, 2.4, 0.0;
    refLinesDomain[3] << 2.4, 0.0, 0.0, 0.0;
}

Eigen::Vector2d ObstacleAvoidance::perpendicular(const Eigen::Vector2d& a) {

    Eigen::Vector2d b(a(1), -a(0));
    return b;
}

double ObstacleAvoidance::det(const Eigen::Matrix2d& mat) {
    return mat.determinant();
}

bool ObstacleAvoidance::checkDirectionVectors(const Eigen::Vector2d& vector1, const Eigen::Vector2d& vector2) {
    Eigen::Vector2d unitVector1 = vector1.normalized();
    Eigen::Vector2d unitVector2 = vector2.normalized();
    double dotProd = unitVector1.dot(unitVector2);
    return dotProd > 0;
}

bool ObstacleAvoidance::checkInDomain(const Eigen::Vector2d& point) {

    if(point[0] > refLinesDomain[0](0,0) && point[0] < refLinesDomain[2](2,0) 
    && point[1] > refLinesDomain[0](0,1) && point[1] < refLinesDomain[1](1,1)) {
        if(obstacles) {
            return false;
        } else {
            return true;
        }
    } else {
        return false;
    }
}

std::pair<int, Eigen::Vector2d> ObstacleAvoidance::lineIntersection(const Eigen::Matrix2d& locations) {

    std::map<int, std::pair<Eigen::Vector2d, double>> potIter;
    Eigen::Vector2d moveVec = locations.col(1) - locations.col(0);
    int count = 0;

    for (const Eigen::Matrix2d& refLine : refLinesDomain) {
        // int countPlus = 0;
        // if (count + 1 > refLinesDomain.rows()) {
        //     countPlus = 0;
        // } else {
        //     countPlus = count;
        // }

        Eigen::Vector2d xdiff;
        xdiff(0) = refLine(0,0) - refLine(1,0);
        xdiff(1) = locations(0,0) - locations(1,0);
        Eigen::Vector2d ydiff;
        ydiff(0) = refLine(0,1) - refLine(1,1);
        ydiff(1) = locations(0,1) - locations(1,1); 
        Eigen::Matrix2d diff;
        diff << xdiff, ydiff;
        // Eigen::Matrix2d refLineMinor;
        // refLineMinor << refLinesDomain.row(count),
        //                   refLinesDomain.row(count + 1);

        double div = det(diff);
        if (div != 0) {
            Eigen::Vector2d d;
            d << det(refLine), det(locations);
            Eigen::Vector2d inter;
            Eigen::Matrix2d temp1;
            temp1 << d, xdiff;
            Eigen::Matrix2d temp2;
            temp2 << d, ydiff;
            inter << det(temp1) / div, det(temp2) / div;
            Eigen::Vector2d interVec = inter - locations.col(0);

            if (checkDirectionVectors(moveVec, interVec)) {
                potIter[count] = std::make_pair(inter, interVec.norm());
            }
            
        }
    }
    //TODO - recheck this if construct
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

    Eigen::Vector2d noObsNewPoint = startPoint + move;

    if (!checkInDomain(startPoint + move)) {
        Eigen::Matrix2d locations;
        locations << startPoint, startPoint + move;
        auto [indexLine, inter] = lineIntersection(locations);
        if (indexLine != -1) {
            Eigen::Matrix2d refLine = refLinesDomain[indexLine];
            Eigen::Vector2d refVec;
            refVec << refLine(1,0) - refLine(0,0), refLine(1,1) - refLine(0,1);
            Eigen::Vector2d perpVec = perpendicular(refVec);
            Eigen::Vector2d newPoint;
            //TODO - check this
            if (!perpVec(0)) {
                newPoint << noObsNewPoint(0), 2*inter(1) - move(1) - startPoint(1);
            } else {
                newPoint << 2*inter(0) - move(0) - startPoint(0), noObsNewPoint(1);
            }

            if (checkInDomain(newPoint)) {
                return std::make_pair(newPoint, true);
            } else {
                Eigen::Vector2d newPointAlt = startPoint - move;
                if (checkInDomain(newPointAlt)) {
                    return std::make_pair(newPointAlt, true);
                } else {
                    return std::make_pair(startPoint, true);
                }
            }
        } else {
            return std::make_pair(startPoint, true);
        }
    } else {
        return std::make_pair(startPoint + move, true);
    }
    // return std::make_pair(Eigen::Vector2d(0.0,0.0), false);
}



