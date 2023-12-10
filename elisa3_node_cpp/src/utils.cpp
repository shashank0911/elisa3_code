#include "utils.h"

Eigen::MatrixXd get_domain() {
    Eigen::MatrixXd domain(4,2);
    domain << 0, 0, 0, 1.15, 2.4, 1.15, 2.4, 0;
    return domain;
}

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

Eigen::VectorXd renew_vec(const Eigen::VectorXd& old_vec) {

    Eigen::VectorXd new_vec(old_vec.size());

    if(old_vec.size() > 0) {
        new_vec.head(old_vec.size() - 1) = old_vec.tail(old_vec.size() - 1);
        new_vec(old_vec.size() - 1) = old_vec(old_vec.size() - 1);
    }

    return new_vec;
}

double yaw_from_quaternion(double x, double y, double z, double w) {
    double t3 = 2.0 * (w * z + x * y);
    double t4 = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(t3, t4);
}

ObstacleAvoidance::ObstacleAvoidance() {
    ref_lines_domain = Eigen::MatrixXd(4,2);
    ref_lines_domain << 0.0, 0.0, 
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

bool ObstacleAvoidance::check_direction_vectors(const Eigen::Vector2d& vector_1, const Eigen::Vector2d& vector_2) {
    Eigen::Vector2d unit_vector_1 = vector_1.normalized();
    Eigen::Vector2d unit_vector_2 = vector_2.normalized();
    double dot_prod = unit_vector_1.dot(unit_vector_2);
    return dot_prod > 0;
}

bool ObstacleAvoidance::check_in_domain(const Eigen::Vector2d& point) {

    if(point[0] > this->ref_lines_domain(0,0) && point[0] < this->ref_lines_domain(2,0) 
    && point[1] > this->ref_lines_domain(0,1) && point[1] < this->ref_lines_domain(1,1)) {
        if(obstacles) {
            return true;
        } else {
            return false;
        }
    }
}

std::pair<int, Eigen::Vector2d> ObstacleAvoidance::line_intersection(const Eigen::Matrix2d& locations) {

    std::map<int, std::tuple<Eigen::Vector2d, double>> pot_iter;
    Eigen::Vector2d move_vec = locations.col(1) - locations.col(0);

    for (int count = 0; count < this->ref_lines_domain.rows(); count++) {
        int count_plus = 0;
        if (count + 1 > this->ref_lines_domain.rows()) {
            count_plus = 0;
        } else {
            count_plus = count;
        }

        Eigen::Vector2d xdiff;
        xdiff(0) = this->ref_lines_domain(count,0) - this->ref_lines_domain(count_plus,0);
        xdiff(1) = locations(0,0) - locations(1,0);
        Eigen::Vector2d ydiff;
        ydiff(0) = this->ref_lines_domain(count,1) - this->ref_lines_domain(count_plus,1);
        ydiff(1) = locations(0,1) - locations(1,1); 
        Eigen::Matrix2d diff;
        diff << xdiff, ydiff;
        Eigen::Matrix2d ref_line_minor;
        ref_line_minor << this->ref_lines_domain.row(count),
                          this->ref_lines_domain.row(count+1);

        double div = this->det(diff);
        if (div != 0) {
            Eigen::Vector2d d;
            d << this->det(ref_line_minor), this->det(locations);
            Eigen::Vector2d inter;
            inter << this->det(Eigen::Matrix2d(d, xdiff)) / div, this->det(Eigen::Matrix2d(d, ydiff)) / div;
            Eigen::Vector2d inter_vec = inter - locations.col(0);

            if (this->check_direction_vectors(move_vec, inter_vec)) {
                pot_iter[count] = std::make_tuple(inter, inter_vec.norm());
            }
            
        }
    }

    if (!pot_iter.empty()) {
        auto key_min = std::min_element(pot_iter.begin(), pot_iter.end(), 
            [](const auto& lhs, const auto& rhs) {
                return std::get<1>(lhs.second) < std::get<1>(rhs.second);
            });
        
        return std::make_pair(key_min->first, std::get<0>(key_min->second));
    } else {
        return std::make_pair(-1, Eigen::Vector2d::Zero());
    }
}

std::pair<Eigen::Vector2d, bool> ObstacleAvoidance::obstacle_avoidance(const Eigen::Vector2d& start_point, const Eigen::Vector2d& move) {

    Eigen::Vector2d no_obs_new_point = start_point + move;

    if (!this->check_in_domain(start_point + move)) {
        auto [index_line, inter] = this->line_intersection(Eigen::Matrix2d(start_point, start_point + move));
        if (index_line != -1) {
            
        }
    }
}



