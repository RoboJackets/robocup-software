#include "GradientAscent1D.hpp"

#include <math.h>
#include <utils.h>

#include <algorithm>
#include <tuple>

GradientAscent1D::GradientAscent1D(Gradient1DConfig* config) : config(config) {
    currentx = config->startX;
    previousx = config->prevX;

    // (*(config->f))
    // value of the function pointer in the config which is also a pointer
    std::tuple<float, float> funcOutput = (*(config->f))(currentx);

    currentVal = std::get<0>(funcOutput);
    currentdx = std::get<1>(funcOutput);
    previousdx = std::get<1>((*(config->f))(previousx));

    temperature = 1;

    iterationCount = 0;
}

bool GradientAscent1D::singleStep() {
    float newX = nextX();
    std::tuple<float, float> funcOutput = (*(config->f))(newX);

    previousx = currentx;
    previousdx = currentdx;

    currentx = newX;
    currentdx = std::get<1>(funcOutput);
    currentVal = std::get<0>(funcOutput);

    // Decrease temperature when derivative flips sign
    if (signum(previousdx) != signum(currentdx)) {
        temperature *= config->temperatureDescent;
    }

    iterationCount++;
    return continueExecution();
}

void GradientAscent1D::execute() {
    while (continueExecution()) {
        singleStep();
    }
}

float GradientAscent1D::getXValue() { return currentx; }

float GradientAscent1D::getValue() { return currentVal; }

bool GradientAscent1D::continueExecution() {
    // dx not low enough?
    bool dx_cont = fabs(currentdx) > config->dxError;
    // temp not low enough?
    bool temp_cont = temperature > config->temperatureMin;
    // Val under max? Max has valid config?
    bool max_cont = (config->maxValue == config->maxThresh) ||
                    (config->maxValue - currentVal) > config->maxThresh;
    // Under iteration count?
    bool iter_cont = iterationCount < config->maxIterations;

    return dx_cont && temp_cont && max_cont && iter_cont;
}

float GradientAscent1D::nextX() {
    float delta_x = currentx - previousx;
    float delta_dx = currentdx - previousdx;
    float gamma =
        fabs(temperature * delta_x * delta_dx / (delta_dx * delta_dx));

    float x_offset = gamma * currentdx;

    // Force within bounds
    x_offset = std::min(config->maxXMovement, x_offset);
    x_offset = std::max(config->maxXMovement * -1, x_offset);

    return currentx + x_offset;
}
