#ifndef RANDOMGENERATOR_H
#define RANDOMGENERATOR_H

#include <assert.h>
#include <random>

namespace sn{

class RandomGenerator
{
public:
    RandomGenerator();

    int uniform(int a, int b);
    double uniform(double a, double b);
    double normal(double mean, double sigma);

};

}
#endif // RANDOMGENERATOR_H
