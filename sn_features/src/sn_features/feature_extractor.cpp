#include <sn_features/feature_extractor.h>

namespace sn{


void l1_normalize(feature_t & data_)
{
    int total = 0;
    for(double d: data_)
        total+=d;
    total*=0.5;
    for(int i=0; i<data_.size(); ++i)
        data_[i]/=total;
}

void l2_normalize(feature_t & data_)
{
    int total = 0;
    for(double d: data_)
        total+=d*d;
    total = std::sqrt(total);
    for(int i=0; i<data_.size(); ++i)
        data_[i]/=total;
}

void inf_normalize(feature_t & data_)
{
    double total = 0;
    for(double d: data_)
        total=std::max(total,d);
    for(int i=0; i<data_.size(); ++i)
        data_[i]/=total;
}

void normalize(feature_t & data_, double norm)
{
    for(int i=0; i<data_.size(); ++i)
        data_[i]/=norm;
}


}
