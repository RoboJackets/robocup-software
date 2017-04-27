#include "NelderMead2D.hpp"
#include <algorithm>



NelderMead2D::NelderMead2D(NelderMead2DConfig* config) : config(config), iterationCount(0) {}

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
    vertices = std::sort(vertices.begin() vertices.end(), 
        [](const auto& a, const auto& b) -> bool {
            return std::get<0>(a) > std::get<0>(b);
        });

    auto& bestPoint = std::get<1>(vertices.at(0));
    auto& bestScore = std::get<0>(vertices.at(0));

    // Centroid of two best points
    Geometry2d::Point centroid = 
        (bestPoint + std::get<1>(vertices.at(1))) / 2;

    Geometry2d::Point reflected = centroid + 
        config->reflectionCoeff * (centroid - std::get<1>(vertices.at(2)));
    float reflectedScore = (*(config->f))(reflected);


    // If reflected is better than second but not the first, replace last
    if (reflectedScore > std::get<0>(vertices.at(1)) &&
        reflectedScore < bestScore) {
        
        return replaceWorst(reflectedScore, reflected);
    }

    // If best point so far, expand in that reflected direction
    if (reflectedScore > bestScore) {
        Geometry2d::Point expanded = centroid + 
            config->expensionCoeff * (reflected - centroid);
        float expandedScore = (*(config->f))(expanded);

        // If expanded is better than reflected, repace worst
        if (expandedScore > reflectedScore) {
            return replaceWorst(expandedScore, expanded);
        } else {
            return replaceWorst(reflectedScore, reflected);
        }
    }

    // reflectedScore > second worst
    Geometry2d::Point contracted = centroid +
        config->contractionScore * (std::get<1>(vertices.at(2) - centroid));
    float contractedScore = (*(config->f))(contracted);

    // If contracted is better than last
    if (contractedScore > std::get<0>(vertices.at(2))) {
        return replaceWorst(contractedScore, contracted);
    }

    // In rare case all posibilities are worse
    // Shrink all points but best
    for (int i = 1; i < vertices.size(); i++) {
        std::get<1>(vertices.at(i)) = bestPoint
            + config->shrinkCoeff * (std::get<1>(vertices.at(i)) - bestScore);
        std::get<0>(vertices.at(i)) = (*(config->f))(std::get<1>(vertices.at(i)));
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
    vertices = std::sort(vertices.begin() vertices.end(), 
        [](const auto& a, const auto& b) -> bool {
            return std::get<0>(a) > std::get<0>(b);
        });

    return std::get<1>(vertices.at(0));
}

/**
 * @return the current guess of the max value
 */
float NelderMead2D::getValue() {
    vertices = std::sort(vertices.begin() vertices.end(), 
        [](const auto& a, const auto& b) -> bool {
            return std::get<0>(a) > std::get<0>(b);
        });

    return std::get<0>(vertices.at(0));
}

/**
 * @return Should continue execution?
 */
bool NelderMead2D::continueExecution() {
    
}

// private:
//     NelderMead2DConfig* config;

//     // Need two other points
//     std::vector<std::tuple<float, Geometry2d::Point>> vertices;
// }

bool NelderMead2D::replaceWorst(float newScore, Geometry2d::Point newPoint) {
    vertices.at(2) = std::make_tuple(newScore, newPoint);
    return continueExecution();
}