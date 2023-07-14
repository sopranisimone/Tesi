#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

#include <iostream>
#include <vector>

class LowPassFilter {

public:

    LowPassFilter(double initial = 0.0, double smoothingFactor = 0.1);
    void filter(double input);
    double getValue() const;

private:

    double currentValue;
    double alpha;

};

#endif // LOWPASSFILTER_H