#ifndef CONFUSION_MATRIX_H
#define CONFUSION_MATRIX_H

#include <map>
#include <fstream>
#include <iostream>

namespace sn {

typedef std::map<std::string, std::map<std::string, double> > ConfusionMatrix;

template<typename Truth, typename Test, typename Sim>
ConfusionMatrix confusion_matrix(Truth const& truth,
                                 Test const& test,
                                 Sim sim){
    // init matrix
    ConfusionMatrix matrix;
    for(auto it: truth)
      for(auto it2: truth)
        matrix[it->name][it2->name]=0;

    // calcul
    for(auto it: test){
      std::string arg_max;
      double max_dist = 0.0;
      for(auto it2: truth){
        double score = sim(it2, it);
        if(score >= max_dist){
          arg_max = it2->name;
          max_dist = score;
        }
      }
      if(max_dist > 0.0 && arg_max.size() > 0){
        matrix[arg_max][it->name] += 1;
      }
    }
    return matrix;
}

double get_precision(const ConfusionMatrix& matrix){

  // precision
  double total = 0;
  double good = 0;
  for(auto it: matrix){
    for(auto it2: it.second){
      if(it.first == it2.first)
        good += it2.second;
      total += it2.second;
    }
  }
  return good/total*100;
}

void save_confusion_matrix(const ConfusionMatrix& matrix, const std::string& filename){
  std::ofstream logger(filename);

  int count1 = 0;
  for(auto it: matrix){
    int count2 = 0;
    for(auto it2: it.second){
      logger << count1 << "\t" << count2 << "\t" << it2.second << std::endl;
      count2++;
    }
    count1++;
  }
}

void print(const ConfusionMatrix& matrix){
    auto pre = std::cout.precision();
    std::cout.precision(0);
    std::cout << std::fixed;
    int count = 0;
    for(auto it: matrix)
      std::cout << it.first << " " << count++ << "\n";
    count = 0;
    for(auto it: matrix)
      std::cout << "\t" << count++;
    std::cout << std::endl;
    count = 0;
    for(auto it: matrix){
      std::cout << count++ << "\t";
      for(auto it2: it.second)
        if(it2.second == 0)
          std::cout << "\t";
        else
          std::cout << it2.second << "\t";
      std::cout << std::endl;
    }
    std::cout.precision(pre);
    std::cout.unsetf ( std::ios::floatfield );
}


}

#endif // CONFUSION_MATRIX_H
