#ifndef POLAR_HISTOGRAM_H
#define POLAR_HISTOGRAM_H

#include <sn_geometry/point.h>

namespace sn {

class PolarHistogram
{

public:
    PolarHistogram() = default;
    PolarHistogram(double size_bin_theta, double size_bin_rho);

    void add(point_t const& pt);
    double get(point_t const& pt)const;


    void l1_normalize();
    void l2_normalize();
    void inf_normalize();
    void normalize(double norm);
    void print();

    std::vector<double> get(){return data_;}

private:
    std::vector<double> data_;
    double size_bin_theta_;
    double size_bin_rho_;

    double bin_factor(double val, double bin_size, int index);

};

}

#endif
