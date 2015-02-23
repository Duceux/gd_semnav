
#include "sn_features/bowtools.h"

#include "fstream"
#include "iostream"
#include "string"
#include "sstream"
//#include "boost/filesystem/v3/operations.hpp"
#include <boost/filesystem/operations.hpp>

opencvBOWForSmallImgs bow;

// usage: create_vocabulary force(bool) feature_width(int) vocabulary_size(int) vocabulary_file(str) feature_file1(str) feature_file2(str) ...
int main ( int argc, char** argv )
{
  if (argc < 6)
  {
    std::cout << "Wrong number of arguments. Usage: create_vocabulary force(bool) feature_width(int) vocabulary_size(int) vocabulary_file(str) feature_file1(str) feature_file2(str) ..." << std::endl;
    return 0;
  }
  bool force;
  std::istringstream(argv[1]) >> force;
  int feature_width;
  std::istringstream(argv[2]) >> feature_width;
  int vocabulary_size;
  std::istringstream(argv[3]) >> vocabulary_size;
  std::string vocabulary_file_path = argv[4];

  if (!force && boost::filesystem::exists(vocabulary_file_path))
  {
    std::cout << "Vocabulary file: " << vocabulary_file_path << " already exists, exiting." << std::endl;
  }
  else
  {
    std::cout << "Loading features from " << argc-4 << " files. ";
    cv::Mat feature_mat(0, 0, CV_32FC1);
    for (int i = 5; i < argc; ++i)
    {
      std::string feature_file_path = argv[i];
      std::ifstream feature_file(feature_file_path.c_str());
      std::string line;
      int line_nbr = 0;
      while (feature_file.good())
      {
        getline(feature_file, line);
        std::stringstream line_stream(line);
        int label;
        int feature_nbr = 0;
        int rd_int;
        char rd_char;
        float rd_flt;
        line_stream >> label;
        while (line_stream.good())
        {
          line_stream >> rd_int;
          if (rd_int == feature_nbr)
          {
            line_stream >> rd_char;
            if (rd_char == ':')
            {
              line_stream >> rd_flt;
              feature_mat.push_back(rd_flt);
            }
            feature_nbr++;
          }
        }
        if (feature_nbr%feature_width != 0)
          std::cout << "In file: " << feature_file_path << ", feature number " << line_nbr << " has " << feature_nbr << "values but should have " << feature_width << "." << std::endl;
        line_nbr++;
      }
    }

    int rows = feature_mat.rows/feature_width;
    // Create vocabulary
    std::cout << "Creating " << vocabulary_size << " entries vocabulary from " << rows << " features...";
    bow.createVoca(feature_mat.reshape(1, rows), vocabulary_size, vocabulary_file_path);
    std::cout << " done!" << std::endl << "Wrote vocabulary to file: " << vocabulary_file_path << '.' << std::endl;
  }

  return 1;
}
