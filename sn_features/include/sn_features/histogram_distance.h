#ifndef HISTOGRAM_DISTANCE_H
#define HISTOGRAM_DISTANCE_H

#include <vector>

namespace sn {

typedef std::vector<double> feature_t;

double euclidean_distance(feature_t l, feature_t r);

double chi2_distance(feature_t l, feature_t r);

double symmetric_chi2_distance(feature_t l, feature_t r);

double batthacharyya_distance(feature_t const& l, feature_t const& r);

double kullback_leibler_distance(feature_t const& l, feature_t const& r);

double jensen_shannon_distance(feature_t const& l, feature_t const& r);

}

#endif // HISTOGRAM_DISTANCE_H
