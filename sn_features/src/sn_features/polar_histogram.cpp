#include <sn_features/polar_histogram.h>

namespace sn {

PolarHistogram::PolarHistogram(double size_bin_theta, double size_bin_rho)
{
    size_bin_theta_ = size_bin_theta;
    size_bin_rho_ = size_bin_rho;
    data_.resize(1000);
}

double PolarHistogram::bin_factor(double val, double bin_size, int index)
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

    int index_theta = angle/size_bin_theta_;
    int nb_bin_theta = 2*M_PI/size_bin_theta_;
//    std::cout << index_theta << " " << size_bin_theta_ << std::endl;

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

void PolarHistogram::l1_normalize()
{
    int total = 0;
    for(double d: data_)
        total+=d;
    for(int i=0; i<data_.size(); ++i)
        data_[i]/=total;
}

void PolarHistogram::l2_normalize()
{
    int total = 0;
    for(double d: data_)
        total+=d*d;
    total = std::sqrt(total);
    for(int i=0; i<data_.size(); ++i)
        data_[i]/=total;
}

void PolarHistogram::inf_normalize()
{
    double total = 0;
    for(double d: data_)
        total=std::max(total,d);
    for(int i=0; i<data_.size(); ++i)
        data_[i]/=total;
}

void PolarHistogram::print()
{
    std::cout.precision(2);
    for(double d: data_)
        std::cout << std::fixed  << d << " ";
    std::cout << std::endl;
}



}
