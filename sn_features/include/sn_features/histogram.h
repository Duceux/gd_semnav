#ifndef HISTOGRAM_H
#define HISTOGRAM_H

namespace sn{

#define SIMPLE 0
#define CIRCULAR 1

template <typename T, typename G>
class Histogram
{
public:
    Histogram(uint size, int BORDER_TYPE, G minval, G maxval);

    ~Histogram();

    T& operator [](uint index){return data_[index+1];}

    uint argMax();
    G valMax();

    G weightedMax(int neighboorsize = 1);

    void add(G val);

    friend  inline std::ostream& operator << (std::ostream& out, const Histogram<T, G>& h){
        out << "(";
        for(uint i=1; i<=h.size_; ++i)
            out << h.data_[i] << ",";
        out << ")";
        return out;
    }

private:
    T* data_;
    int borderType_;
    uint size_;
    G minval_;
    G maxval_;
    G binsize_;

    double bin_factor(double val, double bin_size, int index);
};

template <typename T, typename G>
Histogram<T, G>::Histogram(uint size, int BORDER_TYPE, G minval, G maxval):
    data_(new T[size+2]),
    borderType_(BORDER_TYPE),
    size_(size),
    minval_(minval),
    maxval_(maxval),
    binsize_((maxval-minval)/(T)size)
{
    for(uint i=0; i<=size_+1; ++i)
        data_[i]=0;
}

template <typename T, typename G>
Histogram<T, G>::~Histogram(){
    delete [] data_;
}

template <typename T, typename G>
uint Histogram<T, G>::argMax()
{
    T max = std::numeric_limits<T>::min();
    uint argmax = -1;
    for(uint i=1; i<=size_; ++i)
        if(max < data_[i]){
            argmax = i;
            max = data_[i];
        }
    return argmax-1;
}

template <typename T, typename G>
G Histogram<T, G>::valMax()
{
    T max = std::numeric_limits<T>::min();
    uint argmax = -1;
    for(uint i=1; i<=size_; ++i)
        if(max < data_[i]){
            argmax = i;
            max = data_[i];
        }
    argmax-=1;
    G minr = binsize_*argmax + minval_;
    G maxr = binsize_*(argmax+1) + minval_;
    return (minr+maxr)/2.0;
}

template <typename T, typename G>
double Histogram<T, G>::bin_factor(double val, double bin_size, int index)
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

template <typename T, typename G>
void Histogram<T, G>::add(G val){
    if(val <= minval_ || val > maxval_)return;
    uint index = std::floor( (val-minval_)/binsize_ )+1;
    if(index >= 1 && index <= size_){
        for(int i=index-1; i<=index+1; ++i){
            int idi = i;
            if(idi < 1)
                idi = size_;
            else if(idi > size_)
                idi = 1;
            data_[idi]+=bin_factor(val-minval_, binsize_, i-1);
        }
    }

}

}

#endif // HISTOGRAM_H
