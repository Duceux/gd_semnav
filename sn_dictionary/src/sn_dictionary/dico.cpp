#include <sn_dictionary/dico.h>
#include <sn_msgs/Descriptor.h>

namespace sn{

int FastGetter::operator ()(const std::vector<double> &des, std::vector<std::vector<double> > &words,
                             double th, Distance func)
{
    for(int i=0; i<words.size(); ++i)
        if(func(words[i], des) < th)
            return i;
    words.push_back(des);
    return words.size()-1;
}

}// sn
