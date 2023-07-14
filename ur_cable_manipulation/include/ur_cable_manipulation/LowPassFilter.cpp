#include "LowPassFilter.hpp"

LowPassFilter::LowPassFilter(double initial, double smoothingFactor)
    : currentValue(initial), alpha(smoothingFactor) {}

void LowPassFilter::filter(double input) {
    currentValue = alpha * input + (1 - alpha) * currentValue;
}

double LowPassFilter::getValue() const {
    return currentValue;
}
