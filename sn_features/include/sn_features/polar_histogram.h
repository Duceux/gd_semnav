#ifndef POLAR_HISTOGRAM_H
#define POLAR_HISTOGRAM_H

#include <sn_geometry/point.h>

namespace sn {

class BaseHistogram{
public:
  void l1_normalize();
  void l2_normalize();
  void inf_normalize();
  void normalize(double norm);
  void print();
  std::vector<double> get_feature()const;
protected:
  std::vector<double> data_;
  double bin_factor(double val, double bin_size, int index);

};


class PolarHistogram: public BaseHistogram
{

public:
    PolarHistogram() = default;
    PolarHistogram(double size_bin_theta, double size_bin_rho);

    void add(point_t const& pt);
    double get(point_t const& pt)const;

protected:
    double size_bin_theta_;
    double size_bin_rho_;


};

class Histogram2D: public BaseHistogram
{

public:
    Histogram2D() = default;
    Histogram2D(double size_bin_x, double size_bin_y, double min_val, double max_val):
      size_bin_x_(size_bin_x), size_bin_y_(size_bin_y), min_val_(min_val), max_val_(max_val){
      int nb_bin_x = std::ceil((max_val_-min_val_)/size_bin_x_);
      int nb_bin_y = std::ceil((max_val_-min_val_)/size_bin_y_);
      data_.resize(nb_bin_x*nb_bin_y);
    }

    void add(point_t const& pt);
    double get(point_t const& pt)const;

protected:
    double size_bin_x_;
    double size_bin_y_;
    double min_val_;
    double max_val_;
};

}

#endif
