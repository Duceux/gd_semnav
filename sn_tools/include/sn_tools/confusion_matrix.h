#ifndef CONFUSION_MATRIX_H
#define CONFUSION_MATRIX_H

#include <map>
#include <fstream>
#include <iostream>

namespace sn {

struct ConfusionMatrix{
  std::map<std::string, std::map<std::string, double> > matrix;
  unsigned int nb_queries;

  ConfusionMatrix& operator +=(ConfusionMatrix const& b);
  ConfusionMatrix():nb_queries(0){}
};

template<typename Truth, typename Test, typename Sim>
ConfusionMatrix confusion_matrix(Truth const& truth,
                                 Test const& test,
                                 Sim sim){
    // init matrix
    ConfusionMatrix matrix;
    matrix.nb_queries = test.size();

    for(auto it: truth)
      for(auto it2: truth)
        matrix.matrix[it->name][it2->name]=0;

    // calcul
    for(auto it: test){
      std::string arg_max;
      double max_dist = 0.0;
      for(auto it2: truth){
        double score = sim(it, it2);
        if(score >= max_dist){
          arg_max = it2->name;
          max_dist = score;
        }
      }
      if(max_dist > 0.0 && arg_max.size() > 0){
        matrix.matrix[arg_max][it->name] += 1;
      }
    }
    return matrix;
}

double get_precision(const ConfusionMatrix& matrix){

  // precision
  double good = 0;
  for(auto it: matrix.matrix){
    for(auto it2: it.second){
      if(it.first == it2.first)
        good += it2.second;
    }
  }
  return good/(double)matrix.nb_queries*100.0;
}

double get_precision_ignoring_missed(const ConfusionMatrix& matrix){

  // precision
  double good = 0;
  double total = 0;
  for(auto it: matrix.matrix){
    for(auto it2: it.second){
      if(it.first == it2.first)
        good += it2.second;
      total+=it2.second;
    }
  }
  return good/total*100.0;
}

void save_confusion_matrix(const ConfusionMatrix& matrix, const std::string& filename){
  std::ofstream logger(filename);

  int count1 = 0;
  for(auto it: matrix.matrix){
    int count2 = 0;
    for(auto it2: it.second){
      logger << count1 << "\t" << count2 << "\t" << it2.second << std::endl;
      count2++;
    }
    count1++;
  }
}

void save_confusion_matrix(const ConfusionMatrix& matrix, const std::string& filename,
                           std::map<int, std::string> mapping);

void print(const ConfusionMatrix& matrix){
    auto pre = std::cout.precision();
    std::cout.precision(0);
    std::cout << std::fixed;
    int count = 0;
    for(auto it: matrix.matrix)
      std::cout << it.first << " " << count++ << "\n";
    count = 0;
    for(auto it: matrix.matrix)
      std::cout << "\t" << count++;
    std::cout << std::endl;
    count = 0;
    for(auto it: matrix.matrix){
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
