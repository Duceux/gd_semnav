#include <sn_features/polar_histogram.h>

namespace sn {


PolarHistogram::PolarHistogram(double size_bin_theta, double size_bin_rho)
{
  size_bin_theta_ = size_bin_theta;
  size_bin_rho_ = size_bin_rho;
  data_.resize(1000);
}

double BaseHistogram::bin_factor(double val, double bin_size, int index)
{
  double minr = bin_size*index;
  double maxr = bin_size*(index+1);
  if(val>minr && val<maxr)
    return 1.0;
  if(val<=minr)
    return 1.0 - (minr - val)/bin_size;
  if(val>=maxr)
    return 1.0 - (val - maxr)/bin_size;
}

void PolarHistogram::add(const point_t &pt)
{
  double rho = l2_norm(pt);
  double angle = std::atan2(pt.y, pt.x)+M_PI;
  //    std::cout << angle << " " << rho << std::endl;

  if(angle >= 2.0*M_PI)angle-=2.0*M_PI;
  int index_theta = angle/size_bin_theta_;
  int nb_bin_theta = 2.0*M_PI/size_bin_theta_;

  if(!(index_theta >= 0 && index_theta < nb_bin_theta))
    std::cout << angle << " " << index_theta << " " << size_bin_theta_ << " "<< nb_bin_theta << std::endl;
  assert(index_theta >= 0 && index_theta < nb_bin_theta);

  int index_rho = rho/size_bin_rho_;
  //    std::cout << index_rho << " " << size_bin_rho_ << std::endl;
  assert(index_rho >= 0);
  int overflow = nb_bin_theta+(index_rho+1)*nb_bin_theta - data_.size();
  if(overflow >= 0)
    data_.resize(data_.size()+(overflow+1)*nb_bin_theta);

  //    std::cout << "----" << std::endl;
  for(int i=0; i<3; ++i)
    for(int j=0; j<3; ++j)
    {
      //compute contribution
      //dist avec bin
      int idi = index_theta-1+i;
      if(idi < 0)
        idi = nb_bin_theta-1;
      if(idi >= nb_bin_theta)
        idi = 0;
      int idj = index_rho-1+j;
      if(idj < 0)
        continue;
      data_[idi+idj*nb_bin_theta]+=bin_factor(angle, size_bin_theta_, index_theta-1+i)*bin_factor(rho, size_bin_rho_, idj);
      assert(idi+idj*nb_bin_theta >= 0 && idi+idj*nb_bin_theta < data_.size());
    }

  //    for(double d: data_)
  //        std::cout << d << "\t";
  //    std::cout << std::endl;
}

double PolarHistogram::get(const point_t &pt) const
{
  double rho = l2_norm(pt);
  double angle = std::atan2(pt.y, pt.x)+M_PI;
  if(angle >= 2*M_PI)
    angle=angle-2*M_PI;
  int index_theta = angle/size_bin_theta_;
  int nb_bin_theta = 2*M_PI/size_bin_theta_;
  assert(index_theta >= 0 && index_theta < nb_bin_theta);
  int index_rho = rho/size_bin_rho_;
  assert(index_rho >= 0);
  assert(index_theta+index_rho*nb_bin_theta >= 0);
  if(index_theta+index_rho*nb_bin_theta >= data_.size())
    return 0.0;
  return data_[index_theta+index_rho*nb_bin_theta];
}

void BaseHistogram::l1_normalize()
{
  int total = 0;
  for(double d: data_)
    total+=d;
  total*=0.5;
  for(int i=0; i<data_.size(); ++i)
    data_[i]/=total;
}

void BaseHistogram::l2_normalize()
{
  int total = 0;
  for(double d: data_)
    total+=d*d;
  total = std::sqrt(total);
  for(int i=0; i<data_.size(); ++i)
    data_[i]/=total;
}

void BaseHistogram::inf_normalize()
{
  double total = 0;
  for(double d: data_)
    total=std::max(total,d);
  if(total > 0.0)
    for(int i=0; i<data_.size(); ++i)
      data_[i]/=total;
}

void BaseHistogram::normalize(double norm)
{
  for(int i=0; i<data_.size(); ++i)
    data_[i]/=norm;
}

void BaseHistogram::print()
{
  std::cout.precision(2);
  for(double d: data_)
    std::cout << std::fixed  << d << " ";
  std::cout << std::endl;
}

std::vector<double> BaseHistogram::get_feature() const
{
  return data_;
}

void Histogram2D::add(const point_t &pt)
{
  if(pt.x < min_val_ || pt.x >= max_val_)
    return;
  if(pt.y < min_val_ || pt.y >= max_val_)
    return;

  int index_x = (pt.x-min_val_)/size_bin_x_;
  int index_y = (pt.y-min_val_)/size_bin_y_;
  int nb_bin_x = std::ceil((max_val_-min_val_)/size_bin_x_);
  int nb_bin_y = std::ceil((max_val_-min_val_)/size_bin_y_);
  assert(index_x >= 0 && index_x < nb_bin_x);
  assert(index_y >= 0 && index_y < nb_bin_y);

  for(int i=0; i<3; ++i)
    for(int j=0; j<3; ++j)
    {
      //compute contribution
      //dist avec bin
      int idi = index_y-1+i;
      if(idi < 0 || idi >= nb_bin_y)
        continue;
      int idj = index_x-1+j;
      if(idj < 0 || idj >= nb_bin_x)
        continue;
      data_[idi+idj*nb_bin_y]+=bin_factor(pt.y, size_bin_y_, idi)*bin_factor(pt.x, size_bin_x_, idj);
      if(! (idi+idj*nb_bin_y >= 0 && idi+idj*nb_bin_y < data_.size()))
        std::cout << idi << " " << idj << " " << nb_bin_y << " " << data_.size() << std::endl;
      assert(idi+idj*nb_bin_y >= 0 && idi+idj*nb_bin_y < data_.size());
    }
}

double Histogram2D::get(const point_t &pt) const
{
  if(pt.x < min_val_ || pt.x >= max_val_)
    return 0.0;
  if(pt.y < min_val_ || pt.y >= max_val_)
    return 0.0;

  int index_x = (pt.x-min_val_)/size_bin_x_;
  int index_y = (pt.y-min_val_)/size_bin_y_;
  int nb_bin_x = std::ceil((max_val_-min_val_)/size_bin_x_);
  int nb_bin_y = std::ceil((max_val_-min_val_)/size_bin_y_);
  assert(index_x >= 0 && index_x < nb_bin_x);
  assert(index_y >= 0 && index_y < nb_bin_y);
  return data_[index_y+index_x*nb_bin_y];
}


}
