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

    auto& bestPoint = std::get<1>(vertices.at(0));
    auto& bestScore = std::get<0>(vertices.at(0));

    // Centroid of two best points
    Geometry2d::Point centroid = (bestPoint + std::get<1>(vertices.at(1))) / 2;

    Geometry2d::Point reflected =
        centroid +
        config.reflectionCoeff * (centroid - std::get<1>(vertices.at(2)));
    float reflectedScore = (config.f)(reflected);

    // If reflected is better than second but not the first, replace last
    if (reflectedScore > std::get<0>(vertices.at(1)) &&
        reflectedScore < bestScore) {
        return replaceWorst(reflectedScore, reflected);
    }

    // If best point so far, expand in that reflected direction
    if (reflectedScore > bestScore) {
        Geometry2d::Point expanded =
            centroid + config.expansionCoeff * (reflected - centroid);
        float expandedScore = (config.f)(expanded);

        // If expanded is better than reflected, replace worst
        if (expandedScore > reflectedScore) {
            return replaceWorst(expandedScore, expanded);
        } else {
            return replaceWorst(reflectedScore, reflected);
        }
    }

    // reflectedScore > second worst
    Geometry2d::Point contracted =
        centroid +
        config.contractionCoeff * (std::get<1>(vertices.at(2)) - centroid);
    float contractedScore = (config.f)(contracted);

    // If contracted is better than last
    if (contractedScore > std::get<0>(vertices.at(2))) {
        return replaceWorst(contractedScore, contracted);
    }

    // In rare case all posibilities are worse
    // Shrink all points but best
    for (int i = 1; i < vertices.size(); i++) {
        std::get<1>(vertices.at(i)) =
            bestPoint +
            config.shrinkCoeff * (std::get<1>(vertices.at(i)) - bestScore);
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
    float maxX = 0;
    float maxY = 0;
    for (int i = 0; i < vertices.size(); i++) {
        float dx = (std::get<1>(vertices.at(i)) -
                    std::get<1>(vertices.at((i + 1) % 3))).x();
        float dy = (std::get<1>(vertices.at(i)) -
                    std::get<1>(vertices.at((i + 1) % 3))).y();

        dx = (float)fabs(dx);
        dy = (float)fabs(dy);

        maxX = std::max(dx, maxX);
        maxY = std::max(dy, maxY);
    }

    bool over_min = (maxX > config.minDist.x()) && (maxY > config.minDist.y());
    bool not_near_max =
        (config.maxValue == config.maxThresh) ||
        (config.maxValue - std::get<0>(vertices.at(0)) > config.maxThresh);
    bool under_iter = iterationCount < config.maxIterations;

    return over_min && not_near_max && under_iter;
}

void NelderMead2D::sortVertices() {
    std::sort(vertices.begin(), vertices.end(),
              [](const auto& a, const auto& b)
                  -> bool { return std::get<0>(a) > std::get<0>(b); });
}

bool NelderMead2D::replaceWorst(float newScore, Geometry2d::Point newPoint) {
    vertices.at(2) = std::make_tuple(newScore, newPoint);
    return continueExecution();
}