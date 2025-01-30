#include "gradient_ascent_1d.hpp"

#include <cmath>
#include <tuple>

#include <rj_common/utils.hpp>

GradientAscent1D::GradientAscent1D(Gradient1DConfig* config) : config_(config) {
    currentx_ = config_->start_x;
    previousx_ = config_->prev_x;

    // (*(config_->f))
    // value of the function pointer in the config_ which is also a pointer
    std::tuple<float, float> func_output = (*(config_->f))(currentx_);

    current_val_ = std::get<0>(func_output);
    currentdx_ = std::get<1>(func_output);
    previousdx_ = std::get<1>((*(config_->f))(previousx_));

    temperature_ = 1;

    iteration_count_ = 0;
}

bool GradientAscent1D::single_step() {
    float new_x = next_x();
    std::tuple<float, float> func_output = (*(config_->f))(new_x);

    previousx_ = currentx_;
    previousdx_ = currentdx_;

    currentx_ = new_x;
    currentdx_ = std::get<1>(func_output);
    current_val_ = std::get<0>(func_output);

    // Decrease temperature when derivative flips sign
    if (signum(previousdx_) != signum(currentdx_)) {
        temperature_ *= config_->temperature_descent;
    }

    iteration_count_++;
    return continue_execution();
}

void GradientAscent1D::execute() {
    while (continue_execution()) {
        single_step();
    }
}

float GradientAscent1D::get_x_value() { return currentx_; }

float GradientAscent1D::get_value() { return current_val_; }

bool GradientAscent1D::continue_execution() {
    // dx not low enough?
    bool dx_cont = fabs(currentdx_) > config_->dx_error;
    // temp not low enough?
    bool temp_cont = temperature_ > config_->temperature_min;
    // Val under max? Max has valid config?
    bool max_cont = (config_->max_value == config_->max_thresh) ||
                    (config_->max_value - current_val_) > config_->max_thresh;
    // Under iteration count?
    bool iter_cont = iteration_count_ < config_->max_iterations;

    return dx_cont && temp_cont && max_cont && iter_cont;
}

float GradientAscent1D::next_x() {
    float delta_x = currentx_ - previousx_;
    float delta_dx = currentdx_ - previousdx_;
    float gamma = fabs(temperature_ * delta_x * delta_dx / (delta_dx * delta_dx));

    float x_offset = gamma * currentdx_;

    // Force within bounds
    x_offset = std::min(config_->max_x_movement, x_offset);
    x_offset = std::max(config_->max_x_movement * -1, x_offset);

    return currentx_ + x_offset;
}
