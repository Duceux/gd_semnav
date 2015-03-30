#include <sn_tools/confusion_matrix.h>

namespace sn {

ConfusionMatrix& ConfusionMatrix::operator +=(const ConfusionMatrix &b)
{
  for(auto it: b.matrix)
    for(auto it2: it.second)
      matrix[it.first][it2.first] += it2.second;
  nb_queries+=b.nb_queries;
  return *this;
}

void save_confusion_matrix(const ConfusionMatrix &matrix, const std::string &filename,
                           std::map<int, std::string> mapping)
{
  std::ofstream logger(filename);

  for(auto it: mapping){
    for(auto it2: mapping){
      logger << it.first << "\t" << it2.first << "\t" << matrix.matrix.at(it.second).at(it2.second) << std::endl;
    }
  }
}

}
