#ifndef STRINGSTREAM_H
#define STRINGSTREAM_H

#include <sstream>

namespace sn {

class StringStream{
public:

  template<typename T>
  StringStream& operator <<(T const& t){
    str_ << t;
    return *this;
  }

  operator std::string()const{return str_.str();}

private:
  std::stringstream str_;
};

}

#endif // STRINGSTREAM_H
