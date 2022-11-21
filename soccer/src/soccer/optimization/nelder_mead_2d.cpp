#include "nelder_mead_2d.hpp"

#include <algorithm>
#include <cmath>

NelderMead2D::NelderMead2D(NelderMead2DConfig& config) : config_(config), iteration_count_(0) {
    // Creates starting points at [start], [start] + [-x, y], [start] + [x, y]
    for (int i = -1; i < 2; i++) {
        rj_geometry::Point p =
            config_.start + i * rj_geometry::Point(config_.step.x(), i * config_.step.y());

        vertices_.push_back(std::make_tuple((config_.f)(p), p));
    }
}

/**
 * Runs a single step of the optimization algorithm
 * (One reflection / expansion / contraction / shrink)
 *
 * @note This is most ikely not the function you are looking for to find
 *       the max. This is used when you want to split the optimization
 *       accross multiple time steps
 */
bool NelderMead2D::single_step() {
    // Order vectors by score descending
    sort_vertices();

    auto& best_point = std::get<1>(vertices_.at(0));
    auto& best_score = std::get<0>(vertices_.at(0));

    // Centroid of two best points
    rj_geometry::Point centroid = (best_point + std::get<1>(vertices_.at(1))) / 2;

    rj_geometry::Point reflected =
        centroid + config_.reflection_coeff * (centroid - std::get<1>(vertices_.at(2)));
    float reflected_score = (config_.f)(reflected);

    // If reflected is better than second but not the first, replace last
    if (reflected_score > std::get<0>(vertices_.at(1)) && reflected_score < best_score) {
        return replace_worst(reflected_score, reflected);
    }

    // If best point so far, expand in that reflected direction
    if (reflected_score > best_score) {
        rj_geometry::Point expanded = centroid + config_.expansion_coeff * (reflected - centroid);
        float expanded_score = (config_.f)(expanded);

        // If expanded is better than reflected, replace worst
        if (expanded_score > reflected_score) {
            return replace_worst(expanded_score, expanded);
        } else {
            return replace_worst(reflected_score, reflected);
        }
    }

    // reflected_score > second worst
    rj_geometry::Point contracted =
        centroid + config_.contraction_coeff * (std::get<1>(vertices_.at(2)) - centroid);
    float contracted_score = (config_.f)(contracted);

    // If contracted is better than last
    if (contracted_score > std::get<0>(vertices_.at(2))) {
        return replace_worst(contracted_score, contracted);
    }

    // In rare case all posibilities are worse
    // Shrink all points but best
    for (unsigned long i = 1; i < vertices_.size(); i++) {
        std::get<1>(vertices_.at(i)) =
            best_point + config_.shrink_coeff * (std::get<1>(vertices_.at(i)) - best_score);
        std::get<0>(vertices_.at(i)) = (config_.f)(std::get<1>(vertices_.at(i)));
    }

    iteration_count_++;

    return continue_execution();
}

/**
 * Executes the full optimization algorithm
 */
void NelderMead2D::execute() {
    while (continue_execution()) {
        single_step();
    }
}

/**
 * @return the XY coordinate of the current guess of the max
 */
rj_geometry::Point NelderMead2D::get_point() {
    sort_vertices();

    return std::get<1>(vertices_.at(0));
}

/**
 * @return the current guess of the max value
 */
float NelderMead2D::get_value() {
    sort_vertices();

    return std::get<0>(vertices_.at(0));
}

/**
 * @return Should continue execution?
 */
bool NelderMead2D::continue_execution() {
    sort_vertices();

    // Fit bounding box
    float max_x = 0;
    float max_y = 0;
    for (int i = 0; i < vertices_.size(); i++) {
        float dx = (std::get<1>(vertices_.at(i)) - std::get<1>(vertices_.at((i + 1) % 3))).x();
        float dy = (std::get<1>(vertices_.at(i)) - std::get<1>(vertices_.at((i + 1) % 3))).y();

        dx = (float)fabs(dx);
        dy = (float)fabs(dy);

        max_x = std::max(dx, max_x);
        max_y = std::max(dy, max_y);
    }

    bool over_min = (max_x > config_.min_dist.x()) && (max_y > config_.min_dist.y());
    bool not_near_max = (config_.max_value == config_.max_thresh) ||
                        (config_.max_value - std::get<0>(vertices_.at(0)) > config_.max_thresh);
    bool under_iter = iteration_count_ < config_.max_iterations;

    return over_min && not_near_max && under_iter;
}

void NelderMead2D::sort_vertices() {
    std::sort(vertices_.begin(), vertices_.end(),
              [](const auto& a, const auto& b) -> bool { return std::get<0>(a) > std::get<0>(b); });
}

bool NelderMead2D::replace_worst(float new_score, rj_geometry::Point new_point) {
    vertices_.at(2) = std::make_tuple(new_score, new_point);
    return continue_execution();
}