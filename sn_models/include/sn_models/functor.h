#ifndef FUNCTOR_H
#define FUNCTOR_H

#include <sn_models/graph_of_word.h>
#include <memory>

namespace sn{

class no_type{
public:
  virtual bool operator()(const node_t& n) const{return true;}
  virtual bool operator()(const edge_t& e) const{return true;}
  bool call(const node_t& n) const{return (*this)(n);}
  bool call(const edge_t& n) const{return (*this)(n);}
};

class single_type: public no_type{
public:
  single_type(std::string type, bool inverse):m_type(type), m_inverse(inverse){}
  virtual bool operator()(const node_t& n) const{
    if(m_inverse)
      return n.key.type == m_type;
    else
      return n.key.type != m_type;
  }
  virtual bool operator()(const edge_t& e) const{
    if(m_inverse)
      return e.parent.type == m_type &&
          e.child.type == m_type;
    else
      return e.parent.type != m_type ||
          e.child.type != m_type;
  }

protected:
  bool m_inverse;
  std::string m_type;
};

class edge_type: public no_type{
public:
  edge_type(bool inverse):m_inverse(inverse){}
  virtual bool operator()(const node_t& n) const{return !m_inverse;}
  virtual bool operator()(const edge_t& e) const{return m_inverse;}

protected:
  bool m_inverse;
};

template <typename F>
class functor_wrapper : public no_type {
  std::shared_ptr<F> p;
public:
  functor_wrapper(F const& f):p(new F(f)){}

  virtual bool operator()(const node_t& n) const{return p->call(n);}
  virtual bool operator()(const edge_t& e) const{return p->call(e);}
};

class two_type: public no_type{
public:
  template <typename F, typename T>
  two_type(const F& type1, const T& type2):
    m_filter1(new functor_wrapper<F>(type1)),
    m_filter2(new functor_wrapper<T>(type2))
  {}
  virtual bool operator()(const node_t& n) const{return m_filter1->call(n) && m_filter2->call(n);}
  virtual bool operator()(const edge_t& e) const{return m_filter1->call(e) && m_filter2->call(e);}

  ~two_type() {
    delete m_filter1;
    delete m_filter2;
  }

private:
  no_type* m_filter1;
  no_type* m_filter2;
};

class ruled_two_type: public no_type{
public:
  template <typename F, typename T>
  ruled_two_type(const F& type1, const T& type2, const std::function<bool (bool, bool)>& func_):
    m_filter1(new functor_wrapper<F>(type1)),
    m_filter2(new functor_wrapper<T>(type2)),
    func(func_)
  {}
  virtual bool operator()(const node_t& n) const{return func(m_filter1->call(n), m_filter2->call(n));}
  virtual bool operator()(const edge_t& n) const{return func(m_filter1->call(n), m_filter2->call(n));}

  ~ruled_two_type() {
    delete m_filter1;
    delete m_filter2;
  }

private:
  no_type* m_filter1;
  no_type* m_filter2;
  std::function<bool (bool, bool)> func;
};

}
#endif // FUNCTOR_H
