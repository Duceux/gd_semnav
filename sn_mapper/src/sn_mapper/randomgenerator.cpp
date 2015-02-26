#include <sn_mapper/randomgenerator.h>
#include <random>

namespace sn{

RandomGenerator::RandomGenerator()
{
    /* initialize random seed: */
    srand (time(NULL));
}

int RandomGenerator::uniform(int a, int b){
    static std::default_random_engine gen;
    std::uniform_int_distribution<> dis(a, b);
    return dis(gen);
}

double RandomGenerator::uniform(double a, double b){
    static std::default_random_engine gen;
    std::uniform_real_distribution<> dis(a, b);
    return dis(gen);
}

double RandomGenerator::normal(double mean, double sigma){
    static std::default_random_engine gen;
    std::normal_distribution<double> dis(mean, sigma);
    return dis(gen);
}

}
