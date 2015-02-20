#include <sn_features/histogram_distance.h>
#include <cmath>

namespace sn{

#define PSEUDOZERO 0.0000001

double euclidean_distance(const feature_t &first, const feature_t &last)
{
    auto min_size = std::min(first.size(), last.size());
    double accumulator = 0.;
    for (unsigned int i = 0; i < min_size; i++){
        accumulator += (first[i] - last[i])*(first[i] - last[i]);
    }
    return std::sqrt(accumulator * 0.5);
}

double chi2_distance(const feature_t &first, const feature_t &last)
{
    auto min_size = std::min(first.size(), last.size());
    double accumulator = 0.;
    for (unsigned int i = 0; i < min_size; i++){
        double p = last[i] == 0 ? PSEUDOZERO : last[i];
        double q = first[i] == 0 ? PSEUDOZERO : first[i];
        accumulator += (q - p)*(q - p)/q;
    }
    return accumulator;
}

double symmetric_chi2_distance(const feature_t &first, const feature_t &last)
{
    auto min_size = std::min(first.size(), last.size());
    double accumulator = 0.;
    for (unsigned int i = 0; i < min_size; i++){
        double p = last[i] == 0 ? PSEUDOZERO : last[i];
        double q = first[i] == 0 ? PSEUDOZERO : first[i];
        accumulator += (q - p)*(q - p)/(q + p);
    }
    return 0.5 * accumulator; //TOCHECK Some books says to use 2 other 0.5
}

}
