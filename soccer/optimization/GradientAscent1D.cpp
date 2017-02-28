#include "GradientAscent1D.hpp"
#include <math.h>
#include <algorithm>
#include <tuple>

GradientAscent1D::GradientAscent1D(Gradient1DConfig* config) : config(config){
    currentx  = config->startX;
    previousx = config->prevX;

    std::tuple<float, float> funcOutput = config->f(currentx,  config->args);
    currentVal = std::get<0>(funcOutput);
    currentdx  = std::get<1>(funcOutput);
    previousdx = std::get<1>(config->f(previousx, config->args));

    temperature = 1;

    iterationCount = 0;
}

bool GradientAscent1D::singleStep() {
    float newX = nextX();
    std::tuple<float, float> funcOutput = config->f(newX, config->args);

    previousx  = currentx;
    previousdx = currentdx;

    currentx = newX;
    currentdx  = std::get<1>(funcOutput);
    currentVal = std::get<0>(funcOutput);

    // Decrease temperature when derivative flips sign
    if (sign(previousdx) == -1*sign(currentdx)) {
        temperature *= config->temperatureDescent;
    }

    return continueExecution();
}

void GradientAscent1D::execute() {
    while (continueExecution()) {
        singleStep();

        iterationCount++;
    }
}

float GradientAscent1D::getXValue() {
    return currentx;
}

float GradientAscent1D::getValue() {
    return currentVal;
}

bool GradientAscent1D::continueExecution() {
    // dx not low enough?
    bool dx_invalid = fabs(currentdx) > config->dxError;
    // temp not low enough?
    bool temp_invalid = temperature > config->temperatureMin;
    // Val not almost max?
    bool max_invalid = (config->maxValue - currentVal) > config->maxThresh;
    // Under iteration count?
    bool iter_invalid = iterationCount < config->maxIterations;

    return dx_invalid && temp_invalid && max_invalid && iter_invalid;
}

float GradientAscent1D::nextX() {
    float delta_x = currentx - previousx;
    float delta_dx = currentdx - previousdx;
    float gamma = fabs(temperature * delta_x * delta_dx / (delta_dx*delta_dx));

    float x_offset = gamma * currentdx;

    // Force within bounds
    x_offset = std::min(config->maxXMovement, x_offset);
    x_offset = std::max(config->maxXMovement * -1, x_offset);

    return currentx + x_offset;
}

int GradientAscent1D::sign(float val) {
    return (0.0 < val) - (val < 0.0);
}