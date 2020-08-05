#include "NelderMead2D.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

NelderMead2D::NelderMead2D(NelderMead2DConfig& config)
    : config(config), iterationCount(0) {
    // Creates starting points at [start], [start] + [-x, y], [start] + [x, y]
    for (int i = -1; i < 2; i++) {
        Geometry2d::Point p =
            config.start +
            i * Geometry2d::Point(config.step.x(), i * config.step.y());

        vertices.push_back(std::make_tuple((config.f)(p), p));
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
bool NelderMead2D::singleStep() {
    // Order vectors by score descending
    sortVertices();

    auto& best_point = std::get<1>(vertices.at(0));
    auto& best_score = std::get<0>(vertices.at(0));

    // Centroid of two best points
    Geometry2d::Point centroid = (best_point + std::get<1>(vertices.at(1))) / 2;

    Geometry2d::Point reflected =
        centroid +
        config.reflectionCoeff * (centroid - std::get<1>(vertices.at(2)));
    float reflected_score = (config.f)(reflected);

    // If reflected is better than second but not the first, replace last
    if (reflected_score > std::get<0>(vertices.at(1)) &&
        reflected_score < best_score) {
        return replaceWorst(reflected_score, reflected);
    }

    // If best point so far, expand in that reflected direction
    if (reflected_score > best_score) {
        Geometry2d::Point expanded =
            centroid + config.expansionCoeff * (reflected - centroid);
        float expanded_score = (config.f)(expanded);

        // If expanded is better than reflected, replace worst
        if (expanded_score > reflected_score) {
            return replaceWorst(expanded_score, expanded);
        } else {
            return replaceWorst(reflected_score, reflected);
        }
    }

    // reflectedScore > second worst
    Geometry2d::Point contracted =
        centroid +
        config.contractionCoeff * (std::get<1>(vertices.at(2)) - centroid);
    float contracted_score = (config.f)(contracted);

    // If contracted is better than last
    if (contracted_score > std::get<0>(vertices.at(2))) {
        return replaceWorst(contracted_score, contracted);
    }

    // In rare case all posibilities are worse
    // Shrink all points but best
    for (int i = 1; i < vertices.size(); i++) {
        std::get<1>(vertices.at(i)) =
            best_point +
            config.shrinkCoeff * (std::get<1>(vertices.at(i)) - best_score);
        std::get<0>(vertices.at(i)) = (config.f)(std::get<1>(vertices.at(i)));
    }

    iterationCount++;

    return continueExecution();
}

/**
 * Executes the full optimization algorithm
 */
void NelderMead2D::execute() {
    while (continueExecution()) {
        singleStep();
    }
}

/**
 * @return the XY coordinate of the current guess of the max
 */
Geometry2d::Point NelderMead2D::getPoint() {
    sortVertices();

    return std::get<1>(vertices.at(0));
}

/**
 * @return the current guess of the max value
 */
float NelderMead2D::getValue() {
    sortVertices();

    return std::get<0>(vertices.at(0));
}

/**
 * @return Should continue execution?
 */
bool NelderMead2D::continueExecution() {
    sortVertices();

    // Fit bounding box
    float max_x = 0;
    float max_y = 0;
    for (int i = 0; i < vertices.size(); i++) {
        float dx = (std::get<1>(vertices.at(i)) -
                    std::get<1>(vertices.at((i + 1) % 3)))
                       .x();
        float dy = (std::get<1>(vertices.at(i)) -
                    std::get<1>(vertices.at((i + 1) % 3)))
                       .y();

        dx = (float)fabs(dx);
        dy = (float)fabs(dy);

        max_x = std::max(dx, max_x);
        max_y = std::max(dy, max_y);
    }

    bool over_min =
        (max_x > config.minDist.x()) && (max_y > config.minDist.y());
    bool not_near_max =
        (config.maxValue == config.maxThresh) ||
        (config.maxValue - std::get<0>(vertices.at(0)) > config.maxThresh);
    bool under_iter = iterationCount < config.maxIterations;

    return over_min && not_near_max && under_iter;
}

void NelderMead2D::sortVertices() {
    std::sort(vertices.begin(), vertices.end(),
              [](const auto& a, const auto& b) -> bool {
                  return std::get<0>(a) > std::get<0>(b);
              });
}

bool NelderMead2D::replaceWorst(float new_score, Geometry2d::Point new_point) {
    vertices.at(2) = std::make_tuple(new_score, new_point);
    return continueExecution();
}